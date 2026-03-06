package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.Shooter;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;
import frc.robot.util.HubStateMonitor;
import frc.robot.util.HubStateMonitor.HubState;
import frc.robot.util.ShooterKinematics;
import frc.robot.util.ShooterKinematics.ShooterSetpoint;

/**
 * Full shoot-sequence command.  Coordinates vision, kinematics (including
 * moving-while-shooting compensation for both radial and lateral robot motion),
 * and the Superstructure state machine to score FUEL in the HUB.
 *
 * <h2>Execution Sequence</h2>
 * <ol>
 *   <li><b>Initialize</b> — Verify the HUB is scoring-eligible.  If not, the
 *       command finishes immediately.  Otherwise, request
 *       {@link RobotState#PREPPING_TO_SHOOT}.</li>
 *   <li><b>Execute (each loop)</b>
 *     <ol type="a">
 *       <li>Abort to {@link RobotState#STOWED} if the HUB becomes inactive.</li>
 *       <li>Decompose robot velocity into <em>radial</em> (along turret axis) and
 *           <em>lateral</em> (perpendicular to turret axis) components.</li>
 *       <li>Compute an effective distance that accounts for the robot closing or
 *           opening the gap to the HUB during ball flight.</li>
 *       <li>Recalculate the {@link ShooterSetpoint} — both flywheel RPM <em>and</em>
 *           hood angle — at the effective distance.</li>
 *       <li>Add a turret lead angle (on top of the vision correction) to compensate
 *           for lateral robot motion during ball flight.</li>
 *       <li>Request {@link RobotState#SHOOTING} once all readiness conditions are
 *           met and the distance is in range.</li>
 *     </ol>
 *   </li>
 *   <li><b>End (natural)</b> — Hold {@link RobotState#PREPPING_TO_SHOOT} so the
 *       flywheel stays warm for a rapid follow-up shot.</li>
 *   <li><b>End (interrupted)</b> — Return to {@link RobotState#STOWED}.</li>
 * </ol>
 *
 * <h2>Moving-While-Shooting Compensation</h2>
 *
 * <p>The robot's velocity (from robot-relative chassis speeds) is decomposed into
 * two components along and perpendicular to the current turret axis.  This is correct
 * even when the turret is rotated off-center.
 *
 * <h3>Radial velocity — affects flywheel RPM and hood angle</h3>
 * <pre>
 *   d_eff = d_vision − v_radial × t_flight
 * </pre>
 * <p>If the robot drives <em>toward</em> the HUB ({@code v_radial > 0}), the ball
 * arrives at a shorter distance ({@code d_eff < d_vision}), requiring less flywheel
 * energy and a steeper hood angle.  The reverse applies when driving away.
 * Both flywheel RPM and hood angle are re-computed each loop at {@code d_eff}.
 *
 * <h3>Lateral velocity — affects turret lead angle</h3>
 * <pre>
 *   v_lateral_total = v_lateral_translation + ω × R_turret
 *   δθ = arctan( −v_lateral_total × t_flight / d_vision )
 * </pre>
 * <p>If the robot moves left relative to the turret axis ({@code v_lateral > 0}),
 * the HUB will have drifted right by the time the ball arrives.  The turret leads
 * right ({@code δθ < 0}, CW) to intercept.  Robot spin ({@code ω}) adds an
 * additional tangential velocity at the turret pivot ({@code ω × R}) that
 * displaces the ball in the lateral direction and must also be compensated.
 */
public class ShootCommand extends Command {

    // =========================================================================
    // Dependencies
    // =========================================================================

    private final Superstructure          m_superstructure;
    private final VisionSubsystem         m_vision;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final PhotonVisionSubsystem  m_photonVision;

    // =========================================================================
    // Command State
    // =========================================================================

    /** Last in-range distance used for setpoint computation (clamped, never out-of-range). */
    private double  m_lastDistanceM    = 4.0;
    /** Most recent raw distance from vision this loop (un-clamped; may be out of range). */
    private double  m_rawDistanceM     = 4.0;
    /** True when vision provided a distance measurement during the current execute() loop. */
    private boolean m_rawDistanceValid = false;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Constructs a ShootCommand.
     *
     * @param superstructure The superstructure state machine.
     * @param vision         The vision subsystem (distance, tx, tag selection).
     * @param drivetrain     The swerve drivetrain (robot-relative chassis speeds
     *                       used for moving-while-shooting compensation).
     * @param photonVision   The PhotonVision subsystem (raw-pose hub angle).
     */
    public ShootCommand(Superstructure superstructure,
                        VisionSubsystem vision,
                        CommandSwerveDrivetrain drivetrain,
                        PhotonVisionSubsystem photonVision) {
        m_superstructure = superstructure;
        m_vision         = vision;
        m_drivetrain     = drivetrain;
        m_photonVision   = photonVision;
        addRequirements(superstructure, vision);
    }

    // =========================================================================
    // Command Lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        // Start passing immediately if already in the inactive period;
        // otherwise begin normal hub-tracking prep.
        if (HubStateMonitor.getHubState() == HubState.INACTIVE) {
            m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
        } else {
            m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
        }
    }

    @Override
    public void execute() {
        // Inactive period: aim turret at alliance wall dynamically, let
        // Superstructure handle fixed flywheel/hood/feeder setpoints.
        if (HubStateMonitor.getHubState() == HubState.INACTIVE) {
            m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
            commandTurretToAllianceWall();
            return;
        }

        // --- State recovery --------------------------------------------------
        // If something external moved the Superstructure to STOWED while this
        // command is still running (e.g. TrenchTraversalManager detected the
        // robot near a TRENCH zone, overrode to TRAVERSING_TRENCH, then
        // released to STOWED when the robot's estimated pose left the zone),
        // re-request PREPPING_TO_SHOOT so the shoot sequence can resume.
        // Without this, the command stays in STOWED permanently: handleStowed()
        // stops all actuators each loop, but execute() only transitions from
        // PREPPING_TO_SHOOT — creating a deadlock where the flywheel/hood/turret
        // look ready but the feeder and spindexer never run.
        if (m_superstructure.getState() == RobotState.STOWED) {
            m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
        }

        // =====================================================================
        // PHASE 1 — COMPUTE ALL SETPOINTS
        // =====================================================================

        // --- Update hub distance -------------------------------------------------
        // Priority: PhotonVision → Odometry (Limelight removed — PV is primary).
        //
        // m_rawDistanceM  = actual measurement this loop (may be out of range).
        // m_lastDistanceM = last in-range measurement (used for setpoint computation).
        //
        // Keeping both is critical: isDistanceInRange() MUST use the raw value so
        // the gate correctly blocks shots when the robot drifts outside the range.
        // Using only m_lastDistanceM (which never updates when out-of-range) would
        // make isDistanceInRange() always return true after first lock.
        m_rawDistanceValid = false;
        m_photonVision.getHubDistanceMeters().ifPresentOrElse(
            dist -> {
                m_rawDistanceM     = dist;
                m_rawDistanceValid = true;
                if (dist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                        && dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                    m_lastDistanceM = dist;
                }
            },
            () -> m_vision.getOdometryHubDistanceMeters().ifPresent(dist -> {
                m_rawDistanceM     = dist;
                m_rawDistanceValid = true;
                if (dist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                        && dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                    m_lastDistanceM = dist;
                }
            })
        );

        // --- Raw chassis speeds (no filtering) --------------------------------
        ChassisSpeeds spd = m_drivetrain.getState().Speeds;
        double vx    = spd.vxMetersPerSecond;
        double vy    = spd.vyMetersPerSecond;
        double omega = spd.omegaRadiansPerSecond;

        double chassisSpeedMps = Math.hypot(vx, vy);
        boolean isStationary   = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS;

        double dEff;
        double leadAngleDeg;
        ShooterSetpoint setpoint;

        if (isStationary) {
            dEff         = m_lastDistanceM;
            leadAngleDeg = 0.0;
            setpoint     = ShooterKinematics.calculate(dEff, RobotController.getBatteryVoltage());
        } else {
            double turretRad = Math.toRadians(m_superstructure.getTurretAngleDeg());

            // Velocity at the turret pivot = robot center velocity + ω × offset.
            double vxT = vx - omega * Turret.TURRET_OFFSET_Y_M;
            double vyT = vy + omega * Turret.TURRET_OFFSET_X_M;

            double vRadial  =  vxT * Math.cos(turretRad) + vyT * Math.sin(turretRad);
            double vLateral = -vxT * Math.sin(turretRad) + vyT * Math.cos(turretRad);

            // Compute flight time at current distance, then correct distance for robot motion.
            ShooterSetpoint baseSetpoint = ShooterKinematics.calculate(
                    m_lastDistanceM, RobotController.getBatteryVoltage());
            double tFlight = ShooterKinematics.getFlightTimeSeconds(m_lastDistanceM, baseSetpoint);

            dEff = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                   Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M,
                            m_lastDistanceM - vRadial * tFlight));

            setpoint = ShooterKinematics.calculate(dEff, RobotController.getBatteryVoltage());

            double tFlightFinal = ShooterKinematics.getFlightTimeSeconds(dEff, setpoint);
            leadAngleDeg = (dEff > 0 && tFlightFinal > 0)
                    ? Math.toDegrees(Math.atan2(-vLateral * tFlightFinal, dEff))
                            * Shooter.SOTM_LEAD_ANGLE_SCALAR
                    : 0.0;
        }

        // --- Turret: vision correction + lead angle (0 when stationary) ------
        // Priority: PhotonVision → Odometry (Limelight removed — PV is primary).
        double[] turretTargetDeg = {Double.NaN};
        m_photonVision.getHubAngleDeg().ifPresent(pvAngleDeg -> {
            turretTargetDeg[0] = pvAngleDeg + leadAngleDeg;
        });
        if (Double.isNaN(turretTargetDeg[0])) {
            m_vision.getHubRobotRelativeAngleDeg().ifPresent(robotAngleDeg -> {
                turretTargetDeg[0] = robotAngleDeg + leadAngleDeg;
            });
        }

        // =====================================================================
        // PHASE 2 — SEND ALL SETPOINTS (flywheel, hood, turret) AT ONCE
        // =====================================================================

        m_superstructure.applyShooterSetpoint(setpoint);

        if (!Double.isNaN(turretTargetDeg[0])) {
            m_superstructure.commandTurretAngle(turretTargetDeg[0]);
        }

        // --- Physics validity check -------------------------------------------
        // Verify the computed setpoint will actually clear the front rim AND reach
        // hub center at the current raw distance.  This catches cases where the
        // robot is outside the shootable envelope — using only the range check
        // would miss this because m_lastDistanceM never updates out-of-range so
        // isDistanceInRange() would stay true based on stale data.
        boolean distanceOK = isDistanceInRange();
        // isShootable() is a precomputed table lookup — no simulation at runtime.
        boolean physicsOK  = distanceOK
                && ShooterKinematics.isShootable(
                        m_rawDistanceValid ? m_rawDistanceM : m_lastDistanceM);

        // --- Transition to SHOOTING when all conditions are satisfied --------
        // Stationary: use tight tolerance — no urgency, wait for full precision.
        // Moving: use wider tracking tolerance — setpoints shift every loop so
        // tight tolerance would never be satisfied; the continuous fire gate in
        // handleShooting() catches loops where mechanisms drift.
        boolean inPrepping      = m_superstructure.getState() == RobotState.PREPPING_TO_SHOOT;
        boolean mechanismsReady = isStationary
                ? m_superstructure.isReadyToShoot()
                : m_superstructure.isTrackingSetpoints();
        boolean hubSafe         = HubStateMonitor.isSafeToBeginShot();

        if (inPrepping && mechanismsReady && distanceOK && physicsOK && hubSafe) {
            m_superstructure.requestState(RobotState.SHOOTING);
        }

        // If the shot becomes physically impossible mid-shoot (robot drifted out
        // of range), drop back to PREPPING to stop the feeder immediately.
        if (m_superstructure.getState() == RobotState.SHOOTING && !physicsOK) {
            m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
        }

        // --- Diagnostic telemetry (visible on SmartDashboard) ----------------
        SmartDashboard.putBoolean("Shoot/InPrepping",      inPrepping);
        SmartDashboard.putBoolean("Shoot/MechanismsReady", mechanismsReady);
        SmartDashboard.putBoolean("Shoot/DistanceInRange", distanceOK);
        SmartDashboard.putBoolean("Shoot/PhysicsValid",    physicsOK);
        SmartDashboard.putBoolean("Shoot/HubSafe",         hubSafe);
        SmartDashboard.putString( "Shoot/HubState",        HubStateMonitor.getHubState().name());
        SmartDashboard.putNumber( "Shoot/DistanceM",       m_lastDistanceM);
        SmartDashboard.putNumber( "Shoot/RawDistanceM",    m_rawDistanceM);
        SmartDashboard.putNumber( "Shoot/DeffM",           dEff);
        SmartDashboard.putNumber( "Shoot/LeadAngleDeg",    leadAngleDeg);
        SmartDashboard.putBoolean("Shoot/IsStationary",    chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS);
        // Ball exit angle (= 90° − hood angle) — verify this matches what you
        // physically observe from the robot (slow-motion camera or angle gauge).
        // If it does NOT match, the hoodToBallExitAngleDeg formula is wrong.
        SmartDashboard.putNumber( "Shoot/BallExitAngleDeg", 90.0 - setpoint.hoodAngleDeg());
        SmartDashboard.putNumber( "Shoot/FlywheelRPMCmd",   setpoint.flywheelRPM());
        SmartDashboard.putNumber( "Shoot/HoodAngleCmd",     setpoint.hoodAngleDeg());
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until the operator releases the button (or interrupted).
    }

    /**
     * On natural completion: hold {@link RobotState#PREPPING_TO_SHOOT} so the flywheel
     * stays at speed for a faster follow-up shot.
     * On interruption: return to {@link RobotState#STOWED}.
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_superstructure.requestState(RobotState.STOWED);
        } else {
            m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
        }
    }

    // =========================================================================
    // Private Helpers
    // =========================================================================

    /**
     * Returns whether the current robot-to-hub distance is within the shootable range.
     *
     * <p>Uses {@code m_rawDistanceM} (the actual measurement this loop) when vision
     * provided a fresh reading — this correctly blocks shots when the robot drifts
     * outside the range.  Falls back to {@code m_lastDistanceM} only when no
     * vision measurement arrived this loop (e.g. brief tag loss while tracking).
     */
    private boolean isDistanceInRange() {
        double d = m_rawDistanceValid ? m_rawDistanceM : m_lastDistanceM;
        return d >= SuperstructureConstants.MIN_SHOOT_RANGE_M
            && d <= SuperstructureConstants.MAX_SHOOT_RANGE_M;
    }

    /**
     * Points the turret toward the robot's own alliance wall, dynamically
     * accounting for the robot's current field-relative heading.
     *
     * <p>In WPILib field coordinates, the Blue alliance wall is the -X face
     * (field angle 180°) and the Red alliance wall is the +X face (field angle 0°).
     * Converting to robot-relative: {@code turretAngle = wallFieldAngle - robotHeading}.
     * TurretSubsystem normalizes the result to [-180°, +180°] and clamps to ±175°
     * to protect the cable limits.
     */
    private void commandTurretToAllianceWall() {
        double robotHeadingDeg = m_drivetrain.getState().Pose.getRotation().getDegrees();
        // Blue wall = -X direction (180°), Red wall = +X direction (0°).
        // Default to Blue if alliance is unknown (pre-match / simulation).
        double wallFieldAngleDeg =
                (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                        == DriverStation.Alliance.Red)
                ? 0.0 : 180.0;
        m_superstructure.commandTurretAngle(wallFieldAngleDeg - robotHeadingDeg);
    }
}
