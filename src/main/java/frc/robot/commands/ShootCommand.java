package frc.robot.commands;

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

    // Real dt tracking — avoids fixed 0.020 assumption under scheduler jitter
    private double  m_lastTimestamp     = 0.0;

    // =========================================================================
    // Constructor
    // =========================================================================

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
        m_velSeeded        = false;
        m_alphaFireSeeded  = false;
        m_alphaDotFilt     = 0.0;
        m_lastTimestamp    = Timer.getFPGATimestamp();
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
        double dt  = MathUtil.clamp(now - m_lastTimestamp, 0.010, 0.040);
        m_lastTimestamp = now;

        // --- Hub angle (odometry primary, PhotonVision fallback) ---
        double[] hubBaseAngleDeg = {Double.NaN};
        m_vision.getHubRobotRelativeAngleDeg().ifPresent(a -> hubBaseAngleDeg[0] = a);
        if (Double.isNaN(hubBaseAngleDeg[0])) {
            m_photonVision.getHubAngleDeg().ifPresent(a -> hubBaseAngleDeg[0] = a);
        }

        if (!Double.isNaN(hubBaseAngleDeg[0])) {
            // Fresh vision — update stored angle
            m_lastAlphaNowRad = Math.toRadians(hubBaseAngleDeg[0]);
        } else {
            // Vision dropout: propagate angle forward using gyro omega so the
            // turret doesn't freeze while the robot keeps rotating.
            m_lastAlphaNowRad = MathUtil.angleModulus(m_lastAlphaNowRad - omega * dt);
        }
        double alphaNowRad = m_lastAlphaNowRad;

        // Turret pivot velocity in robot frame (includes ω × offset cross-term)
        double vxT = vx - omega * Turret.TURRET_OFFSET_Y_M;
        double vyT = vy + omega * Turret.TURRET_OFFSET_X_M;

        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS
                            && Math.abs(omega) < 0.15;

        ShooterSetpoint setpoint;
        double alphaFireRad;
        double alphaFutureRad;
        double distanceM;

        // Turret pivot offset in robot frame — vision measures from robot center,
        // but ShooterKinematics needs distance from the turret pivot.
        Translation2d turretOffset = new Translation2d(Turret.TURRET_OFFSET_X_M, Turret.TURRET_OFFSET_Y_M);

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
            Translation2d pivotVel = new Translation2d(vxT, vyT);

            // =================================================================
            // STEP 1 — VISION LATENCY COMPENSATION + TURRET OFFSET CORRECTION
            //
            //   Vision gives hub at (t − latency) in robot-center frame.
            //   rotateBy(−ω·lat) corrects for rotation since capture; .minus()
            //   subtracts pivot translation during that window; final .minus()
            //   converts from robot-center to turret-pivot frame so that
            //   ShooterKinematics receives the true barrel-to-hub distance.
            // =================================================================

            Translation2d hub = new Translation2d(m_lastDistanceM, new Rotation2d(alphaNowRad))
                    .rotateBy(new Rotation2d(-omega * Shooter.SOTM_LATENCY_S))
                    .minus(pivotVel.times(Shooter.SOTM_LATENCY_S))
                    .minus(turretOffset); // robot-center → turret-pivot frame

            // =================================================================
            // STEP 2 — ITERATIVE FUTURE HUB SOLVE (2 iterations)
            //
            //   Predict where hub will be relative to pivot when ball arrives.
            //   Angle from raw (unclamped) future vector; clamp only for lookup.
            // =================================================================

            distanceM = hub.getNorm();
            setpoint  = ShooterKinematics.calculate(distanceM); // clamps internally
            Translation2d futHub = hub;

            for (int i = 0; i < 2; i++) {
                double clampedD = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                                  Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, distanceM));
                double tof = ShooterKinematics.getFlightTimeSeconds(clampedD, setpoint);
                futHub    = hub.minus(pivotVel.times(tof));
                distanceM = futHub.getNorm();
                setpoint  = ShooterKinematics.calculate(distanceM);
            }
            distanceM      = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                            Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, distanceM));
            // =================================================================
            // STEP 3 — VIRTUAL GOAL: aim at future hub, use future distance.
            //
            //   The turret on the moving robot inherits robot velocity.  Ball's
            //   ground-frame velocity = flywheel output + robot velocity.  Aiming
            //   the flywheel at the FUTURE hub position already embeds the correct
            //   lead angle — no additional vector subtraction needed (that would
            //   double-compensate for the same robot motion).
            // =================================================================
            alphaFutureRad = futHub.getAngle().getRadians();
            alphaFireRad   = alphaFutureRad;
        }

        // =====================================================================
        // TURRET ANGULAR VELOCITY FEEDFORWARD
        //
        //   alphaDot captures the rate at which the lead angle changes — e.g.
        //   during acceleration phases where ω + vLateral/d misses the drift.
        //   Seeded on first loop to avoid a spike from an arbitrary initial value.
        // =====================================================================

        double alphaDot;
        if (!m_alphaFireSeeded) {
            alphaDot          = 0.0;
            m_alphaDotFilt    = alphaDot; // seed filter with first value — avoids startup ramp from 0
            m_alphaFireSeeded = true;
        } else {
            alphaDot = MathUtil.angleModulus(alphaFireRad - m_lastAlphaFireRad) / dt;
            alphaDot = MathUtil.clamp(alphaDot,
                    -Shooter.SOTM_MAX_ALPHA_DOT_RAD_PER_S,
                     Shooter.SOTM_MAX_ALPHA_DOT_RAD_PER_S);
            // Low-pass (α=0.2, τ≈80ms) suppresses vision-jitter noise on FF signal
            m_alphaDotFilt += 0.2 * (alphaDot - m_alphaDotFilt);
            alphaDot = m_alphaDotFilt;
        }
        m_lastAlphaFireRad = alphaFireRad;

        // =====================================================================
        // SEND SETPOINTS
        // =====================================================================

        m_superstructure.applyShooterSetpoint(setpoint);
        // Turret aims at alphaFireRad.  Updated every loop so the motor naturally
        // tracks to the correct lead angle — no separate look-ahead needed.
        double vLateral = -vxT * Math.sin(alphaFireRad) + vyT * Math.cos(alphaFireRad);
        m_superstructure.commandTurretAngle(Math.toDegrees(alphaFireRad), vLateral, distanceM,
                                            alphaDot);

        // =====================================================================
        // STATE MACHINE TRANSITIONS
        // =====================================================================

        boolean distanceOK = isDistanceInRange();
        boolean inPrepping = currentState == RobotState.PREPPING_TO_SHOOT;

        boolean isNearlyStationary = chassisSpeedMps < 0.1
                && Math.abs(rawSpd.omegaRadiansPerSecond) < 0.15;
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
