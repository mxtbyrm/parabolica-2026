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
 * <h2>SOTM Algorithm — Exact 2D Vector Decomposition</h2>
 * <ol>
 *   <li>Get the base setpoint at the current vision distance. This defines the
 *       ground-frame velocity the ball must have: horizontal component {@code vhBase}
 *       (toward hub) and vertical component {@code vvBase} (the arc / loft).</li>
 *   <li>The shooter must provide: {@code V_shooter = V_ground_ball − V_robot_pivot}.
 *       Subtracting the turret pivot velocity from the required ground-frame ball
 *       velocity gives the velocity the flywheel must produce.</li>
 *   <li>The fire direction {@code alphaFireRad = atan2(v_sr_y, v_sr_x)} naturally
 *       embeds the lateral lead angle — no separate calculation needed.</li>
 *   <li>The horizontal magnitude {@code vh_fw = |V_shooter_horizontal|} and the
 *       unchanged vertical component {@code vvBase} determine the new RPM and
 *       hood angle — correctly adjusting both for radial <em>and</em> lateral motion.</li>
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
        m_velSeeded   = false;
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

        // Turret pivot velocity in robot frame (includes ω × offset cross-term)
        double vxT = vx - omega * Turret.TURRET_OFFSET_Y_M;
        double vyT = vy + omega * Turret.TURRET_OFFSET_X_M;

        // =====================================================================
        // SOTM — EXACT 2D VECTOR DECOMPOSITION
        //
        //   V_shooter = V_ground_ball - V_robot_pivot
        //
        //   The ball's required ground-frame velocity is fully defined by the
        //   static setpoint at the current distance: horizontal component vhBase
        //   (toward hub) and vertical component vvBase (the arc / loft).
        //   Subtracting the turret pivot velocity gives what the flywheel must
        //   actually supply.  This handles radial AND lateral motion simultaneously
        //   with no iteration and no approximation.
        // =====================================================================

        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS
                            && Math.abs(omega) < 0.3;

        // Base setpoint at actual distance — defines the correct arc (loft) to hub.
        ShooterSetpoint baseSetpoint = ShooterKinematics.calculate(m_lastDistanceM);

        ShooterSetpoint setpoint;
        double alphaFireRad;

        if (isStationary) {
            setpoint     = baseSetpoint;
            alphaFireRad = alphaNowRad;
        } else {
            // Ground-frame velocity the ball must have for the correct arc:
            //   horizontal: vhBase pointed toward the hub
            //   vertical:   vvBase = loft — must be preserved exactly
            double v0Base       = ShooterKinematics.rpmToLaunchSpeed(baseSetpoint.flywheelRPM());
            double exitAngleRad = Math.toRadians(90.0 - baseSetpoint.hoodAngleDeg());
            double vhBase       = v0Base * Math.cos(exitAngleRad);
            double vvBase       = v0Base * Math.sin(exitAngleRad);

            // Required ground-frame ball velocity vector (robot frame, toward hub)
            double v_bg_x = vhBase * Math.cos(alphaNowRad);
            double v_bg_y = vhBase * Math.sin(alphaNowRad);

            // Flywheel must provide: V_shooter = V_ground - V_robot_pivot
            double v_sr_x = v_bg_x - vxT;
            double v_sr_y = v_bg_y - vyT;

            // Fire direction naturally includes lateral lead angle
            double vh_fw = Math.hypot(v_sr_x, v_sr_y);
            alphaFireRad = Math.atan2(v_sr_y, v_sr_x);

            if (vh_fw > 0.5) {
                // Recombine horizontal (adjusted) + vertical (unchanged arc)
                double finalV0      = Math.hypot(vh_fw, vvBase);
                double finalExitDeg = Math.toDegrees(Math.atan2(vvBase, vh_fw));
                double finalHoodDeg = Math.max(Shooter.HOOD_MIN_ANGLE_DEG,
                                      Math.min(Shooter.HOOD_MAX_ANGLE_DEG,
                                               90.0 - finalExitDeg));
                setpoint = new ShooterSetpoint(ShooterKinematics.v0ToRPM(finalV0), finalHoodDeg);
            } else {
                // Robot moving faster than ball toward hub — fall back to base setpoint
                setpoint     = baseSetpoint;
                alphaFireRad = alphaNowRad;
            }
        }

        // =====================================================================
        // SEND SETPOINTS
        // =====================================================================

        m_superstructure.applyShooterSetpoint(setpoint);
        // Turret aims at alphaFireRad.  Updated every loop so the motor naturally
        // tracks to the correct lead angle — no separate look-ahead needed.
        double vLateral = -vxT * Math.sin(alphaFireRad) + vyT * Math.cos(alphaFireRad);
        m_superstructure.commandTurretAngle(Math.toDegrees(alphaFireRad), vLateral, m_lastDistanceM);

        // =====================================================================
        // STATE MACHINE TRANSITIONS
        // =====================================================================

        boolean distanceOK = isDistanceInRange();
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
        SmartDashboard.putBoolean("Shoot/IsStationary",    isStationary);
        SmartDashboard.putNumber( "Shoot/DistanceM",       m_lastDistanceM);
        SmartDashboard.putNumber( "Shoot/RawDistanceM",    m_rawDistanceM);
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
