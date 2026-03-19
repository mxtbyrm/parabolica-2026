package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.Shooter;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;
import frc.robot.util.ShooterKinematics;
import frc.robot.util.ShooterKinematics.ShooterSetpoint;

/**
 * Shoot command with moving-while-shooting (SOTM) compensation.
 *
 * <h2>SOTM Algorithm — Virtual Goal (Latency + Iterative Future Hub)</h2>
 * <ol>
 *   <li><b>Vision latency compensation:</b> Vision gives hub at {@code t − latency}.
 *       Rotate by {@code −ω·lat} (heading change since capture) and subtract pivot
 *       translation {@code vxT·lat, vyT·lat} to get the hub's current robot-relative
 *       position.</li>
 *   <li><b>Iterative future hub solve (2 iterations):</b> The ball takes {@code tof}
 *       seconds in flight.  The hub's robot-relative position shifts by
 *       {@code −pivotVel·tof} during that window.  Two iterations converge because
 *       {@code tof} depends on the setpoint which depends on future distance.</li>
 *   <li><b>Virtual goal aim:</b> Aim the turret at {@code futHub.getAngle()} and spin
 *       to the setpoint for {@code futHub.getNorm()}.  The flywheel output, added to
 *       the robot's own velocity, produces the exact ground-frame ball velocity needed
 *       to reach the hub.  No additional vector subtraction — that would double-
 *       compensate for the same robot motion.</li>
 * </ol>
 */
public class ShootCommand extends Command {

    // =========================================================================
    // Dependencies
    // =========================================================================

    private final Superstructure          m_superstructure;
    private final VisionSubsystem         m_vision;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final PhotonVisionSubsystem   m_photonVision;

    // Optional intake for auto-agitation (null in auto where agitation is not needed).
    // Not added as a subsystem requirement so the roller command can run concurrently.
    private final IntakeSubsystem  m_intake;
    private final BooleanSupplier  m_isIntaking;

    /** Seconds between automatic agitate pulses while shooting. */
    private static final double AGITATE_INTERVAL_S = 1.75;

    /** Duration of each agitate pulse (arm holds at agitate position). */
    private static final double AGITATE_DURATION_S = 0.35;

    /** Roller duty cycle during an agitate pulse — same as IntakeAgitateCommand. */
    private static final double AGITATE_ROLLER_PERCENT = 0.20;

    private final Timer m_agitateIntervalTimer = new Timer();
    private final Timer m_agitatePhaseTimer    = new Timer();
    private boolean     m_agitating            = false;
    private boolean     m_wasInShooting        = false; // edge-detect SHOOTING entry

    // =========================================================================
    // Command State
    // =========================================================================

    private double  m_lastDistanceM    = 4.0;
    private double  m_rawDistanceM     = 4.0;
    private boolean m_rawDistanceValid = false;
    private double  m_lastAlphaNowRad  = 0.0; // last known hub angle — never falls back to turret

    private int m_physicsNotOkCount = 0;
    private static final int PHYSICS_NOT_OK_DROP_LOOPS = 5; // ~100 ms

    // EMA for chassis velocity (α = SOTM_VEL_ALPHA, τ ≈ 39 ms at 20 ms loop)
    private double  m_filtVx    = 0.0;
    private double  m_filtVy    = 0.0;
    private double  m_filtOmega = 0.0;
    private boolean m_velSeeded = false;

    // Turret angular velocity feedforward — tracks lead-angle rate of change
    private double  m_lastAlphaFireRad  = 0.0;
    private boolean m_alphaFireSeeded   = false;
    private double  m_alphaDotFilt      = 0.0; // low-pass on alphaDot to suppress jitter
    private double  m_tofFilt           = 0.0; // low-pass on time-of-flight to suppress jitter

    // Real dt tracking — avoids fixed 0.020 assumption under scheduler jitter
    private double  m_lastTimestamp     = 0.0;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Teleop constructor — includes auto-agitation of the intake while shooting.
     *
     * @param intake     Intake subsystem (NOT added as a requirement — roller command
     *                   may run concurrently without cancelling this command).
     * @param isIntaking Returns {@code true} while the driver or operator holds the
     *                   roller button; suppresses agitation when {@code true}.
     */
    public ShootCommand(Superstructure superstructure,
                        VisionSubsystem vision,
                        CommandSwerveDrivetrain drivetrain,
                        PhotonVisionSubsystem photonVision,
                        IntakeSubsystem intake,
                        BooleanSupplier isIntaking) {
        m_superstructure = superstructure;
        m_vision         = vision;
        m_drivetrain     = drivetrain;
        m_photonVision   = photonVision;
        m_intake         = intake;
        m_isIntaking     = isIntaking;
        addRequirements(superstructure, vision);
    }

    /**
     * Auto constructor — no agitation (intake is managed by the auto sequence).
     */
    public ShootCommand(Superstructure superstructure,
                        VisionSubsystem vision,
                        CommandSwerveDrivetrain drivetrain,
                        PhotonVisionSubsystem photonVision) {
        this(superstructure, vision, drivetrain, photonVision, null, () -> false);
    }

    // =========================================================================
    // Command Lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        m_physicsNotOkCount = 0;
        m_velSeeded        = false;
        m_alphaFireSeeded  = false;
        m_alphaDotFilt     = 0.0;
        m_tofFilt          = 0.0;
        m_lastTimestamp    = Timer.getFPGATimestamp();
        m_agitating        = false;
        m_wasInShooting    = false;
        m_agitateIntervalTimer.stop();
        m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
    }

    @Override
    public void execute() {
        // State recovery: STOWED or TRAVERSING_TRENCH → re-request PREPPING
        RobotState currentState = m_superstructure.getState();
        if (currentState == RobotState.STOWED
                || currentState == RobotState.TRAVERSING_TRENCH) {
            m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
        }

        // =====================================================================
        // SENSOR INPUTS
        // =====================================================================

        // --- Distance from vision (odometry primary, PhotonVision fallback) ---
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

        // --- EMA velocity filter ---
        ChassisSpeeds rawSpd = m_drivetrain.getState().Speeds;
        if (!m_velSeeded) {
            m_filtVx    = rawSpd.vxMetersPerSecond;
            m_filtVy    = rawSpd.vyMetersPerSecond;
            m_filtOmega = rawSpd.omegaRadiansPerSecond;
            m_velSeeded = true;
        } else {
            m_filtVx    += Shooter.SOTM_VEL_ALPHA   * (rawSpd.vxMetersPerSecond     - m_filtVx);
            m_filtVy    += Shooter.SOTM_VEL_ALPHA   * (rawSpd.vyMetersPerSecond     - m_filtVy);
            m_filtOmega += Shooter.SOTM_OMEGA_ALPHA * (rawSpd.omegaRadiansPerSecond - m_filtOmega);
        }
        double vx             = m_filtVx;
        double vy             = m_filtVy;
        double omega          = m_filtOmega;
        double chassisSpeedMps = Math.hypot(vx, vy);

        // --- Real dt (FPGA-accurate; guards against scheduler jitter) ---
        double now = Timer.getFPGATimestamp();
        double dt  = MathUtil.clamp(now - m_lastTimestamp, 0.005, 0.040);
        m_lastTimestamp = now;

        // --- Hub angle (odometry primary, PhotonVision fallback) ---
        double[] hubBaseAngleDeg = {Double.NaN};
        m_vision.getHubRobotRelativeAngleDeg().ifPresent(a -> hubBaseAngleDeg[0] = a);
        if (Double.isNaN(hubBaseAngleDeg[0])) {
            m_photonVision.getHubAngleDeg().ifPresent(a -> hubBaseAngleDeg[0] = a);
        }

        if (!Double.isNaN(hubBaseAngleDeg[0])) {
            // Fresh vision — update stored angle and distance.
            m_lastAlphaNowRad = Math.toRadians(hubBaseAngleDeg[0]);
        } else {
            // Vision dropout: propagate BOTH angle and distance using odometry so
            // that TOF and lead-angle stay physically consistent while vision is lost.
            //
            // Angle: rotate by −ω·dt (robot turned, hub appears to move opposite).
            m_lastAlphaNowRad = MathUtil.angleModulus(m_lastAlphaNowRad - omega * dt);
            // Distance: d_dot = −dot(pivotVel, hubUnitVec).
            // Positive dot = pivot moving toward hub → distance shrinks.
            double vxT0 = vx - omega * Turret.TURRET_OFFSET_Y_M;
            double vyT0 = vy + omega * Turret.TURRET_OFFSET_X_M;
            double dRadial = vxT0 * Math.cos(m_lastAlphaNowRad)
                           + vyT0 * Math.sin(m_lastAlphaNowRad);
            // Only floor-clamp: robot genuinely can close to MIN range while moving.
            // Upper clamp is not applied — distance can drift above MAX temporarily
            // without loss (calculate() clamps internally; isDistanceInRange() gates shots).
            m_lastDistanceM = Math.max(
                    m_lastDistanceM - dRadial * dt,
                    SuperstructureConstants.MIN_SHOOT_RANGE_M);
        }
        double alphaNowRad = m_lastAlphaNowRad;

        // Turret pivot velocity in robot frame (includes ω × offset cross-term).
        // Used for the future-hub solve (pivot moves with this velocity during tof).
        double vxT = vx - omega * Turret.TURRET_OFFSET_Y_M;
        double vyT = vy + omega * Turret.TURRET_OFFSET_X_M;

        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS
                    && Math.abs(omega) < 0.05;
                    
        ShooterSetpoint setpoint;
        double alphaFireRad;
        double alphaFutureRad;
        double distanceM;

        // Turret pivot offset in robot frame — vision measures from robot center,
        // but ShooterKinematics needs distance from the turret pivot.
        Translation2d turretOffset = new Translation2d(Turret.TURRET_OFFSET_X_M, Turret.TURRET_OFFSET_Y_M);
        // robotVel: pure robot-center velocity (no offset cross-term).
        // pivotVel: pivot velocity including omega×offset — used only in future-hub solve.
        Translation2d robotVel = new Translation2d(vx, vy);
        Translation2d pivotVel = new Translation2d(vxT, vyT);

        if (isStationary) {
            // Stationary: skip latency/future correction — no movement to predict.
            // Convert robot-center hub vector to turret-pivot frame before using distance.
            Translation2d hubVec = new Translation2d(m_lastDistanceM, new Rotation2d(alphaNowRad))
                    .minus(turretOffset);
            distanceM      = hubVec.getNorm();
            setpoint       = ShooterKinematics.calculate(distanceM);
            alphaFutureRad = hubVec.getAngle().getRadians();
            alphaFireRad   = alphaFutureRad;
        } else {
            // =================================================================
            // STEP 1 — VISION LATENCY COMPENSATION + TURRET OFFSET CORRECTION
            //
            //   Vision gives hub at (t − latency) in robot-center frame.
            //   Latency correction uses robotVel (robot-center velocity) — the
            //   omega×offset cross-term must NOT be included here because the
            //   offset is separately subtracted as a fixed position below.
            //   Using pivotVel here would double-count the offset effect.
            // =================================================================
            Translation2d hub = new Translation2d(m_lastDistanceM, new Rotation2d(alphaNowRad))
                    .rotateBy(new Rotation2d(-omega * Shooter.SOTM_LATENCY_S)) // frame correction first
                    .minus(robotVel.times(Shooter.SOTM_LATENCY_S))             // robot-center translation only
                    .minus(turretOffset);                                        // robot-center → pivot frame (once)

            // =================================================================
            // STEP 2 — TOF FILTER (once, before iterations)
            //
            //   Filter is applied exactly once per execute() loop so the EMA
            //   alpha is correct and the tof used in both iterations is identical
            //   (deterministic convergence, no double filter-state update).
            // =================================================================
            distanceM = hub.getNorm();
            setpoint  = ShooterKinematics.calculate(distanceM);
            double clampedD0 = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                               Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, distanceM));
            double rawTof = ShooterKinematics.getFlightTimeSeconds(clampedD0, setpoint);
            // Adaptive reset threshold: tighter at close range (fast tof changes),
            // looser at long range (slow tof changes from distance drift).
            double resetThreshold = MathUtil.clamp(0.1 + 0.02 * distanceM, 0.1, 0.25);
            if (m_tofFilt == 0.0 || Math.abs(rawTof - m_tofFilt) > resetThreshold) {
                m_tofFilt = rawTof; // large jump or first loop → reset
            } else {
                double tofAlpha = MathUtil.clamp(0.45 - 0.04 * distanceM, 0.2, 0.45);
                m_tofFilt += tofAlpha * (rawTof - m_tofFilt);
            }
            double tof = m_tofFilt;

            // =================================================================
            // STEP 3 — ITERATIVE FUTURE HUB SOLVE (2 iterations)
            //
            //   tof is seeded from the EMA filter (stable, no jitter) but IS
            //   updated inside the loop so each iteration uses a tof consistent
            //   with the refined distance — properly converging the nonlinear
            //   (distance, tof) system.  The filter state is NOT touched here.
            // =================================================================
            Translation2d futHub = hub;
            for (int i = 0; i < 2; i++) {
                futHub    = hub.minus(pivotVel.times(tof));
                distanceM = futHub.getNorm();
                setpoint  = ShooterKinematics.calculate(distanceM);
                double clampedD = MathUtil.clamp(distanceM,
                        SuperstructureConstants.MIN_SHOOT_RANGE_M,
                        SuperstructureConstants.MAX_SHOOT_RANGE_M);
                tof = ShooterKinematics.getFlightTimeSeconds(clampedD, setpoint);
            }

            // =================================================================
            // STEP 4 — VIRTUAL GOAL: aim at future hub, use future distance.
            //
            //   distanceM is left raw (unclamped) here so commandTurretAngle
            //   receives the true distance for the tracking-rate calculation
            //   (ω + vLateral/d).  ShooterKinematics.calculate() already clamped
            //   internally when computing setpoint above.
            //
            //   SOTM_LEAD_ANGLE_SCALAR scales the lateral lead angle so operators
            //   can correct persistent left/right error without recompiling.
            //   SOTM_RADIAL_SCALE scales the radial distance correction so
            //   operators can correct persistent over/under-range error.
            //   Both default to 1.0 (no change from the raw SOTM output).
            // =================================================================
            alphaFutureRad = futHub.getAngle().getRadians();
            double hubAnglePivotRad = hub.getAngle().getRadians();
            double leadDelta        = MathUtil.angleModulus(alphaFutureRad - hubAnglePivotRad);
            alphaFireRad = MathUtil.angleModulus(hubAnglePivotRad
                    + leadDelta * Shooter.SOTM_LEAD_ANGLE_SCALAR);
            distanceM = hub.getNorm() + (distanceM - hub.getNorm()) * Shooter.SOTM_RADIAL_SCALE;
            setpoint  = ShooterKinematics.calculate(MathUtil.clamp(distanceM,
                    SuperstructureConstants.MIN_SHOOT_RANGE_M,
                    SuperstructureConstants.MAX_SHOOT_RANGE_M));
        }

        // vLateral: pivot-frame lateral velocity perpendicular to hub direction.
        // Computed before alphaDot so the model term (ω + vLateral/d) can use it.
        double vLateral = pivotVel.getX() * -Math.sin(alphaFireRad)
                        + pivotVel.getY() *  Math.cos(alphaFireRad);

        // =====================================================================
        // TURRET ANGULAR VELOCITY FEEDFORWARD
        //
        //   alphaDot captures the rate at which the lead angle changes — e.g.
        //   during acceleration phases where ω + vLateral/d misses the drift.
        //   Seeded on first loop to avoid a spike from an arbitrary initial value.
        // =====================================================================

        double alphaDot;
        if (!m_alphaFireSeeded) {
            alphaDot           = 0.0;
            m_alphaDotFilt     = alphaDot; 
            m_lastAlphaFireRad = alphaFireRad; 
            m_alphaFireSeeded  = true;
        } else {
            // Measured alphaDot: numerical derivative of the fire angle.
            // Dynamic Feedforward Blending
            double rawAlphaDot = MathUtil.angleModulus(alphaFireRad - m_lastAlphaFireRad);
            double measuredAlphaDot = MathUtil.clamp(
                    rawAlphaDot / dt, // Using true dt here
                    -Shooter.SOTM_MAX_ALPHA_DOT_RAD_PER_S,
                    Shooter.SOTM_MAX_ALPHA_DOT_RAD_PER_S);

            double modelAlphaDot = MathUtil.clamp(
                    omega + vLateral / Math.max(distanceM, 0.1),
                    -Shooter.SOTM_MAX_ALPHA_DOT_RAD_PER_S,
                    Shooter.SOTM_MAX_ALPHA_DOT_RAD_PER_S);

            double blend = MathUtil.clamp(0.7 - 0.1 * chassisSpeedMps, 0.4, 0.7);
            alphaDot = blend * modelAlphaDot + (1.0 - blend) * measuredAlphaDot;

            double alpha = MathUtil.clamp(0.35 - 0.04 * distanceM, 0.1, 0.35);
            m_alphaDotFilt += alpha * (alphaDot - m_alphaDotFilt);
            alphaDot = m_alphaDotFilt;
        }
        m_lastAlphaFireRad = alphaFireRad;

        // =====================================================================
        // SEND SETPOINTS
        // =====================================================================

        m_superstructure.applyShooterSetpoint(setpoint);
        m_superstructure.commandTurretAngle(Math.toDegrees(alphaFireRad), vLateral, distanceM,
                                            alphaDot);

        // =====================================================================
        // STATE MACHINE TRANSITIONS
        // =====================================================================

        boolean distanceOK = isDistanceInRange();
        boolean inPrepping = currentState == RobotState.PREPPING_TO_SHOOT;

        boolean isNearlyStationary = chassisSpeedMps < 0.1
                && Math.abs(omega) < 0.15;
        boolean mechanismsReady = isNearlyStationary
                ? m_superstructure.isReadyToShoot()
                : m_superstructure.isTrackingSetpoints();

        if (inPrepping && mechanismsReady && distanceOK) {
            m_superstructure.requestState(RobotState.SHOOTING);
        }

        // Drop back if distance leaves range (debounced to avoid glitch drop-back)
        if (currentState == RobotState.SHOOTING && !distanceOK) {
            if (++m_physicsNotOkCount >= PHYSICS_NOT_OK_DROP_LOOPS) {
                m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
                m_physicsNotOkCount = 0;
            }
        } else {
            m_physicsNotOkCount = 0;
        }

        // =====================================================================
        // AUTO AGITATE
        //
        //   While shooting, periodically jostle the intake arm to help balls
        //   enter the funnel.  Suppressed whenever the operator or driver holds
        //   the roller button — in that case the arm stays deployed for normal
        //   intake operation and the agitate cycle resets.
        // =====================================================================
        if (m_intake != null) {
            boolean inShooting = (currentState == RobotState.SHOOTING);

            // Detect entry into SHOOTING — start the agitate interval from zero.
            if (inShooting && !m_wasInShooting) {
                m_agitating = false;
                m_agitateIntervalTimer.restart();
            }
            m_wasInShooting = inShooting;

            if (!inShooting) {
                // Not shooting yet — cancel any in-progress agitate pulse and
                // leave the arm deployed so it is ready when SHOOTING begins.
                if (m_agitating) {
                    m_intake.deploy();
                    m_intake.stopRoller();
                    m_agitating = false;
                }
            } else if (m_isIntaking.getAsBoolean()) {
                // Roller held — stay deployed, stop any agitate roller, reset
                // cycle so agitation starts fresh after the user releases.
                m_intake.deploy();
                m_intake.stopRoller();
                m_agitating = false;
                m_agitateIntervalTimer.restart();
            } else if (m_agitating) {
                // Hold agitate position and run roller at agitate percent.
                m_intake.agitate();
                m_intake.runRollerAt(AGITATE_ROLLER_PERCENT);
                if (m_agitatePhaseTimer.hasElapsed(AGITATE_DURATION_S)) {
                    m_intake.deploy();
                    m_intake.stopRoller();
                    m_agitating = false;
                    m_agitateIntervalTimer.restart();
                }
            } else {
                // Waiting — fire next agitate pulse when interval elapses.
                if (m_agitateIntervalTimer.hasElapsed(AGITATE_INTERVAL_S)) {
                    m_intake.agitate();
                    m_intake.runRollerAt(AGITATE_ROLLER_PERCENT);
                    m_agitating = true;
                    m_agitatePhaseTimer.restart();
                }
            }
        }

        // =====================================================================
        // TELEMETRY
        // =====================================================================

        SmartDashboard.putBoolean("Shoot/InPrepping",       inPrepping);
        SmartDashboard.putBoolean("Shoot/MechanismsReady", mechanismsReady);
        SmartDashboard.putBoolean("Shoot/DistanceInRange",  distanceOK);
        SmartDashboard.putBoolean("Shoot/IsStationary",     isStationary);
        SmartDashboard.putNumber( "Shoot/DistanceM",        m_lastDistanceM);
        SmartDashboard.putNumber( "Shoot/DistanceFutureM",  distanceM);
        SmartDashboard.putNumber( "Shoot/RawDistanceM",     m_rawDistanceM);
        SmartDashboard.putNumber( "Shoot/AlphaNowDeg",      Math.toDegrees(alphaNowRad));
        SmartDashboard.putNumber( "Shoot/AlphaFutureDeg",   Math.toDegrees(alphaFutureRad));
        SmartDashboard.putNumber( "Shoot/AlphaFireDeg",     Math.toDegrees(alphaFireRad));
        SmartDashboard.putNumber( "Shoot/OmegaRadPerSec",   omega);
        SmartDashboard.putNumber( "Shoot/AlphaDotRadPerSec", alphaDot);
        SmartDashboard.putNumber( "Shoot/BallExitAngleDeg", 90.0 - setpoint.hoodAngleDeg());
        SmartDashboard.putNumber( "Shoot/FlywheelRPMCmd",   setpoint.flywheelRPM());
        SmartDashboard.putNumber( "Shoot/HoodAngleCmd",     setpoint.hoodAngleDeg());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (m_intake != null) {
            m_intake.stopRoller();
            m_intake.deploy(); // Leave arm at deployed so intake is ready immediately.
        }
        if (interrupted) {
            m_superstructure.requestState(RobotState.STOWED);
        } else {
            m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
        }
    }

    // =========================================================================
    // Private Helpers
    // =========================================================================

    private boolean isDistanceInRange() {
        double d = m_rawDistanceValid ? m_rawDistanceM : m_lastDistanceM;
        return d >= SuperstructureConstants.MIN_SHOOT_RANGE_M
            && d <= SuperstructureConstants.MAX_SHOOT_RANGE_M;
    }
}
