package frc.robot.commands;

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
 * Shoot command with moving-while-shooting (SOTM) compensation.
 *
 * <h2>SOTM Algorithm</h2>
 * <ol>
 *   <li>Get the base setpoint at the current vision distance and use its
 *       flight time as a prediction horizon.</li>
 *   <li>Predict the hub's position relative to the turret pivot at fire time:
 *       {@code fire = hub_now − pivotVelocity × tof}.
 *       This is a pure position prediction — no velocity decomposition.</li>
 *   <li>Compute the final setpoint at the effective distance
 *       {@code |fire|}.  The turret aims at {@code atan2(fireY, fireX)},
 *       which naturally embeds the lateral lead angle — no separate lead
 *       calculation needed.</li>
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
        m_velSeeded = false;
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
            double aV    = Shooter.SOTM_VEL_ALPHA;
            m_filtVx    += aV * (rawSpd.vxMetersPerSecond     - m_filtVx);
            m_filtVy    += aV * (rawSpd.vyMetersPerSecond     - m_filtVy);
            m_filtOmega += aV * (rawSpd.omegaRadiansPerSecond - m_filtOmega);
        }
        double vx             = m_filtVx;
        double vy             = m_filtVy;
        double omega          = m_filtOmega;
        double chassisSpeedMps = Math.hypot(vx, vy);

        // --- Hub angle (odometry primary, PhotonVision fallback) ---
        double[] hubBaseAngleDeg = {Double.NaN};
        m_vision.getHubRobotRelativeAngleDeg().ifPresent(a -> hubBaseAngleDeg[0] = a);
        if (Double.isNaN(hubBaseAngleDeg[0])) {
            m_photonVision.getHubAngleDeg().ifPresent(a -> hubBaseAngleDeg[0] = a);
        }

        if (!Double.isNaN(hubBaseAngleDeg[0])) {
            m_lastAlphaNowRad = Math.toRadians(hubBaseAngleDeg[0]);
        }
        double alphaNowRad = m_lastAlphaNowRad;

        // Hub position in robot frame (relative to turret pivot)
        double hubX = m_lastDistanceM * Math.cos(alphaNowRad);
        double hubY = m_lastDistanceM * Math.sin(alphaNowRad);

        // Turret pivot velocity in robot frame (includes ω × offset cross-term)
        double vxT = vx - omega * Turret.TURRET_OFFSET_Y_M;
        double vyT = vy + omega * Turret.TURRET_OFFSET_X_M;

        // =====================================================================
        // SOTM — POSITION PREDICTION
        //
        // Mirror of the HTML simulator's fireBall() / updateUI() SOTM logic:
        //
        //   Step 1: base setpoint at current distance → flight time
        //   Step 2: predict hub position at fire time
        //              fire = hub_now − pivotVelocity × tof
        //   Step 3: final setpoint at effective distance |fire|
        //   Step 4: turret aims at atan2(fireY, fireX) — lead is implicit
        // =====================================================================

        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS
                            && Math.abs(omega) < 0.3;

        ShooterSetpoint setpoint;
        double dFire;
        double alphaFireRad;
        boolean dFireValid = true; // true when unclamped dFire is within shoot range

        if (isStationary) {
            dFire        = m_lastDistanceM;
            alphaFireRad = alphaNowRad;
            setpoint     = ShooterKinematics.calculate(dFire);
        } else {
            // Step 1: flight time from base setpoint at current distance
            // Include radial velocity so tof isn't over-estimated when approaching the hub.
            double vRadialNow = vxT * Math.cos(alphaNowRad) + vyT * Math.sin(alphaNowRad);
            ShooterSetpoint baseSetpoint = ShooterKinematics.calculate(m_lastDistanceM);
            double tof = ShooterKinematics.getFlightTimeSeconds(m_lastDistanceM, baseSetpoint, vRadialNow);

            // Step 2: predicted hub position relative to pivot at fire time
            // SOTM_DRAG_DECAY_FACTOR < 1.0 corrects for drag partially dissipating
            // the robot-velocity contribution during flight (vacuum assumption error).
            double fireX = hubX - vxT * tof * Shooter.SOTM_DRAG_DECAY_FACTOR;
            double fireY = hubY - vyT * tof * Shooter.SOTM_DRAG_DECAY_FACTOR;

            // Step 3: effective distance and turret direction
            double rawDFire = Math.hypot(fireX, fireY);
            dFire        = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                           Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, rawDFire));
            alphaFireRad = Math.atan2(fireY, fireX);
            dFireValid   = rawDFire >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                        && rawDFire <= SuperstructureConstants.MAX_SHOOT_RANGE_M;

            // Step 4: final setpoint
            setpoint = ShooterKinematics.calculate(dFire);
        }

        // =====================================================================
        // SEND SETPOINTS
        // =====================================================================

        m_superstructure.applyShooterSetpoint(setpoint);
        // No feedforward (0.0) — lateral lead is already embedded in alphaFireRad.
        m_superstructure.commandTurretAngle(Math.toDegrees(alphaFireRad), 0.0, dFire);

        // =====================================================================
        // STATE MACHINE TRANSITIONS
        // =====================================================================

        boolean distanceOK = isDistanceInRange() && dFireValid;
        boolean inPrepping = currentState == RobotState.PREPPING_TO_SHOOT;

        boolean isNearlyStationary = chassisSpeedMps < 0.1
                && Math.abs(rawSpd.omegaRadiansPerSecond) < 0.3;
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

        SmartDashboard.putBoolean("Shoot/InPrepping",      inPrepping);
        SmartDashboard.putBoolean("Shoot/MechanismsReady", mechanismsReady);
        SmartDashboard.putBoolean("Shoot/DistanceInRange", distanceOK);
        SmartDashboard.putBoolean("Shoot/DFireValid",      dFireValid);
        SmartDashboard.putBoolean("Shoot/IsStationary",    isStationary);
        SmartDashboard.putNumber( "Shoot/DistanceM",       m_lastDistanceM);
        SmartDashboard.putNumber( "Shoot/RawDistanceM",    m_rawDistanceM);
        SmartDashboard.putNumber( "Shoot/DFireM",          dFire);
        SmartDashboard.putNumber( "Shoot/AlphaFireDeg",    Math.toDegrees(alphaFireRad));
        SmartDashboard.putNumber( "Shoot/AlphaNowDeg",     Math.toDegrees(alphaNowRad));
        SmartDashboard.putNumber( "Shoot/OmegaRadPerSec",  omega);
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
