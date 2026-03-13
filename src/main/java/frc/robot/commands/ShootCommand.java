package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    /**
     * Consecutive loops where physicsOK has been false while in SHOOTING.
     * Drop-back to PREPPING_TO_SHOOT only fires after {@link #PHYSICS_NOT_OK_DROP_LOOPS}
     * consecutive failures, preventing single-loop distance glitches from cutting
     * the feeder during rapid robot movement.
     */
    private int m_physicsNotOkCount = 0;
    private static final int PHYSICS_NOT_OK_DROP_LOOPS = 5; // ~100 ms

    // Low-pass filters for chassis speeds used in SOTM.
    // tau=0.05 s smooths wheel-encoder noise while staying responsive to speed changes.
    private final LinearFilter m_vxFilter    = LinearFilter.singlePoleIIR(0.05, 0.02);
    private final LinearFilter m_vyFilter    = LinearFilter.singlePoleIIR(0.05, 0.02);
    private final LinearFilter m_omegaFilter = LinearFilter.singlePoleIIR(0.05, 0.02);


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
        m_physicsNotOkCount = 0;
        m_vxFilter.reset();
        m_vyFilter.reset();
        m_omegaFilter.reset();
        m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
    }

    @Override
    public void execute() {
        // --- State recovery --------------------------------------------------
        // Two recovery cases while this command is still running:
        // 1. STOWED: TrenchTraversalManager overrode to TRAVERSING_TRENCH then
        //    released — without recovery the command deadlocks (handleStowed()
        //    stops actuators every loop; execute() never transitions out).
        // 2. TRAVERSING_TRENCH: robot was in the approach zone when the operator
        //    pressed shoot — recover to PREPPING so scoring can begin.
        RobotState currentState = m_superstructure.getState();
        if (currentState == RobotState.STOWED
                || currentState == RobotState.TRAVERSING_TRENCH) {
            m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
        }

        // =====================================================================
        // PHASE 1 — COMPUTE ALL SETPOINTS
        // =====================================================================

        // --- Update hub distance -------------------------------------------------
        // Priority: Odometry (fused pose estimator) → PhotonVision raw PnP fallback.
        //
        // Consistent with angle priority.  Odometry is primary because the fused
        // pose already incorporates all PV corrections via the Kalman filter and
        // is updated every loop from the turret pivot position.  PhotonVision raw
        // distance is a fallback for pre-match / simulation (unknown alliance).
        //
        // m_rawDistanceM  = actual measurement this loop (may be out of range).
        // m_lastDistanceM = last in-range measurement (used for setpoint computation).
        //
        // Keeping both is critical: isDistanceInRange() MUST use the raw value so
        // the gate correctly blocks shots when the robot drifts outside the range.
        // Using only m_lastDistanceM (which never updates when out-of-range) would
        // make isDistanceInRange() always return true after first lock.
        m_rawDistanceValid = false;
        m_vision.getFusedHubDistanceMeters().ifPresentOrElse(
            dist -> {
                m_rawDistanceM     = dist;
                m_rawDistanceValid = true;
                if (dist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                        && dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                    m_lastDistanceM = dist;
                }
            },
            () -> m_photonVision.getHubDistanceMeters().ifPresent(dist -> {
                m_rawDistanceM     = dist;
                m_rawDistanceValid = true;
                if (dist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                        && dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                    m_lastDistanceM = dist;
                }
            })
        );


        ChassisSpeeds rawSpd = m_drivetrain.getState().Speeds;
        double vx    = m_vxFilter.calculate(rawSpd.vxMetersPerSecond);
        double vy    = m_vyFilter.calculate(rawSpd.vyMetersPerSecond);
        double omega = m_omegaFilter.calculate(rawSpd.omegaRadiansPerSecond);

        double chassisSpeedMps = Math.hypot(vx, vy);

        // --- Hub base angle (without lead) — computed FIRST so it can be used
        // as turretRad in the velocity decomposition below.  Using the current
        // turret angle (mid-slew) would give a wrong vLateral and cause large
        // lead-angle spikes at certain slew positions.
        // Priority: Odometry → PhotonVision fallback.
        double[] hubBaseAngleDeg = {Double.NaN};
        m_vision.getHubRobotRelativeAngleDeg().ifPresent(a -> hubBaseAngleDeg[0] = a);
        if (Double.isNaN(hubBaseAngleDeg[0])) {
            m_photonVision.getHubAngleDeg().ifPresent(a -> hubBaseAngleDeg[0] = a);
        }

        double dEff;
        double leadAngleDeg;
        ShooterSetpoint setpoint;
        double vRadialDbg = 0.0; // for telemetry only

        // WRAPAROUND: turret is mid-slew — skip SOTM, use static setpoints.
        boolean isWrapping = m_superstructure.getState() == RobotState.WRAPAROUND;

        if (isWrapping) {
            dEff         = m_lastDistanceM;
            leadAngleDeg = 0.0;
            setpoint     = ShooterKinematics.calculate(dEff);
        } else {
            // SOTM always active — no hard speed deadband.
            // At near-zero speed vLateral ≈ 0 so lead angle ≈ 0 naturally,
            // avoiding the hard turret jump at the old 0.5 m/s threshold.
            double turretRad = Double.isNaN(hubBaseAngleDeg[0])
                    ? Math.toRadians(m_superstructure.getTurretAngleDeg())
                    : Math.toRadians(hubBaseAngleDeg[0]);

            double vxT = vx - omega * Turret.TURRET_OFFSET_Y_M;
            double vyT = vy + omega * Turret.TURRET_OFFSET_X_M;

            double vRadial  =  vxT * Math.cos(turretRad) + vyT * Math.sin(turretRad);
            vRadialDbg = vRadial;
            double vLateral = -vxT * Math.sin(turretRad) + vyT * Math.cos(turretRad);

            ShooterSetpoint baseSetpoint = ShooterKinematics.calculate(m_lastDistanceM);
            double tFlight = ShooterKinematics.getFlightTimeSeconds(m_lastDistanceM, baseSetpoint, vRadial);

            dEff = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                   Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M,
                            m_lastDistanceM - vRadial * tFlight));

            setpoint = ShooterKinematics.calculate(dEff);

            double tFlightFinal = ShooterKinematics.getFlightTimeSeconds(dEff, setpoint, vRadial);
            leadAngleDeg = (dEff > 0 && tFlightFinal > 0)
                    ? Math.toDegrees(Math.atan2(-vLateral * tFlightFinal, dEff))
                            * Shooter.SOTM_LEAD_ANGLE_SCALAR
                    : 0.0;
        }

        // --- Turret: hub base angle + lead angle ----------------------------
        double[] turretTargetDeg = {Double.NaN};
        if (!Double.isNaN(hubBaseAngleDeg[0])) {
            turretTargetDeg[0] = hubBaseAngleDeg[0] + leadAngleDeg;
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
        // Nearly stationary (< 0.1 m/s): use tight tolerance for precise first shot.
        // Moving: use wider tracking tolerance — setpoint shifts every loop during
        // SOTM so the 1° static tolerance would prevent firing entirely.
        boolean inPrepping         = currentState == RobotState.PREPPING_TO_SHOOT;
        boolean isNearlyStationary = chassisSpeedMps < 0.1;
        boolean mechanismsReady    = isNearlyStationary
                ? m_superstructure.isReadyToShoot()
                : m_superstructure.isFlywheelTracking();
        if (inPrepping && mechanismsReady && distanceOK && physicsOK) {
            m_superstructure.requestState(RobotState.SHOOTING);
        }

        // Drop back to PREPPING only after physicsOK has been false for several
        // consecutive loops.  A single bad distance reading while moving fast
        // would otherwise cut the feeder unnecessarily.
        if (currentState == RobotState.SHOOTING && !physicsOK) {
            if (++m_physicsNotOkCount >= PHYSICS_NOT_OK_DROP_LOOPS) {
                m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
                m_physicsNotOkCount = 0;
            }
        } else {
            m_physicsNotOkCount = 0;
        }

        // --- Diagnostic telemetry (visible on SmartDashboard) ----------------
        SmartDashboard.putBoolean("Shoot/InPrepping",      inPrepping);
        SmartDashboard.putBoolean("Shoot/MechanismsReady", mechanismsReady);
        SmartDashboard.putBoolean("Shoot/DistanceInRange", distanceOK);
        SmartDashboard.putBoolean("Shoot/PhysicsValid",    physicsOK);
        SmartDashboard.putNumber( "Shoot/DistanceM",       m_lastDistanceM);
        SmartDashboard.putNumber( "Shoot/RawDistanceM",    m_rawDistanceM);
        SmartDashboard.putNumber( "Shoot/DeffM",           dEff);
        SmartDashboard.putNumber( "Shoot/VRadialMps",      vRadialDbg);
        SmartDashboard.putNumber( "Shoot/LeadAngleDeg",    leadAngleDeg);
        SmartDashboard.putBoolean("Shoot/IsNearlyStationary", isNearlyStationary);
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
}
