package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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

    private double          m_lastDistanceM = 4.0; // safe starting assumption
    private ShooterSetpoint m_lastSetpoint;

    /**
     * Slew-rate-limited flywheel RPM sent to the superstructure.
     *
     * <p>Tracks {@code m_lastSetpoint.flywheelRPM()} but may only <em>decrease</em>
     * at {@link Shooter#FLYWHEEL_SLEW_RATE_DOWN_RPM_PER_S}.  This prevents the
     * dEff correction from instantly slashing the setpoint when the robot approaches
     * the HUB quickly, keeping the flywheel within {@code isFlywheelAtSpeed()} tolerance
     * so shoot-while-moving can actually fire.  Spin-up is unlimited.
     */
    private double m_smoothedFlywheelRPM = 0.0;

    // --- Velocity EMA state (shoot-on-the-move smoothing) --------------------
    private double m_filtVx    = 0.0;
    private double m_filtVy    = 0.0;
    private double m_filtOmega = 0.0;
    private boolean m_velSeeded = false;

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
        m_lastSetpoint = ShooterKinematics.calculate(m_lastDistanceM);
        m_smoothedFlywheelRPM = m_lastSetpoint.flywheelRPM();
        m_velSeeded = false; // re-seed velocity filter on each activation
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
        // Priority:
        //   1) PhotonVision   — EMA-filtered PnP distance (primary)
        //   2) Odometry       — fused estimator distance (always available)
        //   3) Limelight      — tag-based distance (last-resort fallback)
        // This ensures setpoints stay current even when PhotonVision tags are
        // temporarily occluded.  Odometry is always available when the alliance
        // is known.
        m_photonVision.getHubDistanceMeters().ifPresentOrElse(
            dist -> {
                if (dist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                        && dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                    m_lastDistanceM = dist;
                }
            },
            () -> m_vision.getOdometryHubDistanceMeters().ifPresentOrElse(
                dist -> {
                    if (dist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                            && dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                        m_lastDistanceM = dist;
                    }
                },
                () -> m_vision.getDistanceToHubMeters().ifPresent(dist -> {
                    if (dist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                            && dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                        m_lastDistanceM = dist;
                    }
                })
            )
        );

        // --- Velocity smoothing (EMA low-pass filter) -------------------------
        // Raw ChassisSpeeds jitter due to encoder quantization and CAN latency.
        // A simple exponential moving average stabilises the d_eff correction and
        // lead angle, reducing shot-to-shot scatter while moving.
        ChassisSpeeds rawSpeeds = m_drivetrain.getState().Speeds;
        double alpha = Shooter.SOTM_VELOCITY_ALPHA;
        if (!m_velSeeded) {
            m_filtVx    = rawSpeeds.vxMetersPerSecond;
            m_filtVy    = rawSpeeds.vyMetersPerSecond;
            m_filtOmega = rawSpeeds.omegaRadiansPerSecond;
            m_velSeeded = true;
        } else {
            m_filtVx    += alpha * (rawSpeeds.vxMetersPerSecond    - m_filtVx);
            m_filtVy    += alpha * (rawSpeeds.vyMetersPerSecond    - m_filtVy);
            m_filtOmega += alpha * (rawSpeeds.omegaRadiansPerSecond - m_filtOmega);
        }

        double chassisSpeedMps = Math.hypot(m_filtVx, m_filtVy);
        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS;

        double dEff;
        double leadAngleDeg;

        if (isStationary) {
            // --- Stationary path: raw distance, no SOTM compensation ---------
            // Skip d_eff correction, lead angle, and slew-rate limiting.
            // Setpoints are rock-steady when the robot is parked.
            dEff = m_lastDistanceM;
            leadAngleDeg = 0.0;
            m_lastSetpoint = ShooterKinematics.calculate(dEff);
            m_smoothedFlywheelRPM = m_lastSetpoint.flywheelRPM();
        } else {
            // --- Moving path: full SOTM compensation -------------------------
            double turretRad = Math.toRadians(m_superstructure.getTurretAngleDeg());

            // Velocity at the turret pivot = robot center velocity + ω × offset.
            double vxTurret = m_filtVx - m_filtOmega * Turret.TURRET_OFFSET_Y_M;
            double vyTurret = m_filtVy + m_filtOmega * Turret.TURRET_OFFSET_X_M;

            double vRadial  =  vxTurret * Math.cos(turretRad) + vyTurret * Math.sin(turretRad);
            double vLateral = -vxTurret * Math.sin(turretRad) + vyTurret * Math.cos(turretRad);

            // 2-pass d_eff convergence
            double tFlight1 = (m_lastSetpoint != null)
                    ? ShooterKinematics.getFlightTimeSeconds(m_lastDistanceM, m_lastSetpoint)
                    : 0.0;
            double dEff1 = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                           Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M,
                                    m_lastDistanceM - vRadial * tFlight1));
            ShooterSetpoint pass1Setpoint = ShooterKinematics.calculate(dEff1);
            double tFlight2 = ShooterKinematics.getFlightTimeSeconds(dEff1, pass1Setpoint);
            dEff = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                   Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M,
                            m_lastDistanceM - vRadial * tFlight2));

            m_lastSetpoint = ShooterKinematics.calculate(dEff);

            // Flywheel slew-rate limiting (down only)
            double rawRPM     = m_lastSetpoint.flywheelRPM();
            double maxDropRPM = Shooter.FLYWHEEL_SLEW_RATE_DOWN_RPM_PER_S * 0.020;
            if (rawRPM < m_smoothedFlywheelRPM - maxDropRPM) {
                m_smoothedFlywheelRPM -= maxDropRPM;
            } else {
                m_smoothedFlywheelRPM = rawRPM;
            }

            // Lead angle
            double tFlight = ShooterKinematics.getFlightTimeSeconds(dEff, m_lastSetpoint);
            leadAngleDeg = (dEff > 0 && tFlight > 0)
                    ? Math.toDegrees(Math.atan2(-vLateral * tFlight, dEff))
                        * Shooter.SOTM_LEAD_ANGLE_SCALAR
                    : 0.0;
        }

        // --- Turret: vision correction + lead angle (0 when stationary) ------
        // Priority:
        //   1) PhotonVision   — raw PnP-pose hub angle (EMA-filtered, primary)
        //   2) Odometry       — fused estimator pose (always available)
        //   3) Limelight tx   — direct camera feedback (last-resort fallback)
        double[] turretTargetDeg = {Double.NaN};
        m_photonVision.getHubAngleDeg().ifPresent(pvAngleDeg -> {
            turretTargetDeg[0] = pvAngleDeg + leadAngleDeg;
        });
        if (Double.isNaN(turretTargetDeg[0])) {
            m_vision.getHubRobotRelativeAngleDeg().ifPresent(robotAngleDeg -> {
                turretTargetDeg[0] = robotAngleDeg + leadAngleDeg;
            });
        }
        if (Double.isNaN(turretTargetDeg[0])) {
            m_vision.getTargetTxDeg().ifPresent(tx -> {
                turretTargetDeg[0] = m_superstructure.getTurretAngleDeg() - tx + leadAngleDeg;
            });
        }

        // =====================================================================
        // PHASE 2 — SEND ALL SETPOINTS (flywheel, hood, turret) AT ONCE
        // =====================================================================

        m_superstructure.applyShooterSetpoint(
                new ShooterSetpoint(m_smoothedFlywheelRPM, m_lastSetpoint.hoodAngleDeg()));

        if (!Double.isNaN(turretTargetDeg[0])) {
            m_superstructure.commandTurretAngle(turretTargetDeg[0]);
        }

        // --- Transition to SHOOTING when all conditions are satisfied --------
        // Uses isReadyToShoot() (tight tolerance) so we never enter SHOOTING
        // with mechanisms still converging.  The slew-rate-limited flywheel
        // setpoint and EMA-smoothed velocity ensure mechanisms can track within
        // tight tolerance even while driving at full speed.
        boolean inPrepping      = m_superstructure.getState() == RobotState.PREPPING_TO_SHOOT;
        boolean mechanismsReady = m_superstructure.isReadyToShoot();
        boolean distanceOK      = isDistanceInRange();
        boolean hubSafe         = HubStateMonitor.isSafeToBeginShot();

        if (inPrepping && mechanismsReady && distanceOK && hubSafe) {
            m_superstructure.requestState(RobotState.SHOOTING);
        }

        // --- Diagnostic telemetry (visible on SmartDashboard) ----------------
        // Shows exactly which condition is blocking the PREPPING→SHOOTING
        // transition, eliminating guesswork during field testing.
        SmartDashboard.putBoolean("Shoot/InPrepping",      inPrepping);
        SmartDashboard.putBoolean("Shoot/MechanismsReady", mechanismsReady);
        SmartDashboard.putBoolean("Shoot/DistanceInRange", distanceOK);
        SmartDashboard.putBoolean("Shoot/HubSafe",         hubSafe);
        SmartDashboard.putString( "Shoot/HubState",        HubStateMonitor.getHubState().name());
        SmartDashboard.putNumber( "Shoot/DistanceM",       m_lastDistanceM);
        SmartDashboard.putBoolean("Shoot/IsStationary",    chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS);
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

    /** Returns whether the last known vision distance is within the configured shooting range. */
    private boolean isDistanceInRange() {
        return m_lastDistanceM >= SuperstructureConstants.MIN_SHOOT_RANGE_M
            && m_lastDistanceM <= SuperstructureConstants.MAX_SHOOT_RANGE_M;
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
