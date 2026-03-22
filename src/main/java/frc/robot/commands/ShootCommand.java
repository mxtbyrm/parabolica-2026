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
 * <h2>SOTM Algorithm — Exact Vector Subtraction (EVS)</h2>
 * <ol>
 *   <li><b>Vision latency compensation:</b> Vision gives hub at {@code t − latency}.
 *       Rotate by {@code −ω·lat} (heading change since capture) and subtract robot-center
 *       translation {@code vRobot·lat} to get the hub's current robot-relative position.</li>
 *   <li><b>Exact vector subtraction:</b> Compute the required ground-frame horizontal
 *       velocity to reach the hub from a stationary shot ({@code vHoriz} toward hub).
 *       Subtract robot velocity: {@code V_turret = V_ground_required − V_robot}.
 *       Aim the turret at {@code V_turret}'s direction.</li>
 *   <li><b>Vertical component fixed:</b> The vertical speed {@code vy0} and hood angle
 *       come from the stationary-shot table at the <em>actual</em> hub distance — not
 *       a virtual distance.  This keeps the rim-clearance trajectory correct regardless
 *       of robot speed.  Only the horizontal component (and therefore RPM) changes with
 *       robot velocity.  No tof iteration is needed.</li>
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
    private boolean     m_holdingAgitate       = false; // stay at agitate pos after pulse ends
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
        m_lastTimestamp    = Timer.getFPGATimestamp();
        m_agitating        = false;
        m_holdingAgitate   = false;
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

        // INTAKE_UNSAFE: state machine holds turret at 0° and blocks all setpoints.
        // Skip the full SOTM pipeline until the rack clears the agitate position.
        if (currentState == RobotState.INTAKE_UNSAFE) return;

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
            // Distance: d_dot = −dot(pivotVel, hubUnitVec).  pivotVel here uses
            // TURRET_OFFSET cross-term (small distance-rate error acceptable during dropout).
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

        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS
                    && Math.abs(omega) < 0.05;

        ShooterSetpoint setpoint;
        double alphaFireRad;
        double alphaFutureRad;
        double distanceM;

        // pivotVel: turret pivot velocity in robot frame (includes ω×offset cross-term).
        // Used for EVS velocity subtraction — ball is launched from the pivot, so
        // ground-frame ball velocity = V_turret + V_pivot (not V_robot_center).
        // Note: vision (getFusedHubDistanceMeters / getHubRobotRelativeAngleDeg) already
        // measures from the turret pivot — no offset subtraction needed here.
        Translation2d robotVel  = new Translation2d(vx, vy);
        Translation2d pivotVel  = new Translation2d(
                vx - omega * Turret.TURRET_OFFSET_Y_M,
                vy + omega * Turret.TURRET_OFFSET_X_M);

        // hubPivot: hub position in turret-pivot frame — set in both branches so that
        // the alphaDot model (below) can use hub coordinates without re-computing.
        // Vision already returns pivot-relative distance/angle, so no offset subtraction needed.
        Translation2d hubPivot;

        if (isStationary) {
            // Stationary: vision already gives hub relative to turret pivot — use directly.
            hubPivot       = new Translation2d(m_lastDistanceM, new Rotation2d(alphaNowRad));
            distanceM      = m_lastDistanceM;
            setpoint       = ShooterKinematics.calculate(distanceM);
            alphaFutureRad = alphaNowRad;
            alphaFireRad   = alphaFutureRad;
        } else {
            // =================================================================
            // STEP 1 — VISION LATENCY COMPENSATION
            //
            //   Vision gives hub at (t − latency) already in turret-pivot frame.
            //   Latency correction rotates the stale vector by (−ω·lat) for heading
            //   change, then subtracts robot-center translation (robotVel·lat).
            //   pivotVel is NOT used here — the offset cross-term is second-order
            //   and < 1 cm at typical speeds; robotVel is the correct translation term.
            // =================================================================
            hubPivot = new Translation2d(m_lastDistanceM, new Rotation2d(alphaNowRad))
                    .rotateBy(new Rotation2d(-omega * Shooter.SOTM_LATENCY_S))
                    .minus(robotVel.times(Shooter.SOTM_LATENCY_S));

            // =================================================================
            // STEP 2 — EXACT VECTOR SUBTRACTION (EVS)
            //
            //   Key insight: the ball's vertical trajectory is independent of
            //   horizontal robot motion.  A stationary shot at the actual hub
            //   distance gives the correct vy0 (vertical speed) and hood angle
            //   for rim clearance.  We keep vy0 fixed and only adjust the
            //   horizontal component by subtracting pivot velocity.
            //
            //   Ground-frame ball velocity = V_turret + V_pivot
            //   ∴  V_turret = V_required_ground − V_pivot
            //
            //   No tof iteration needed: vy0 is fixed → tof is fixed by
            //   vertical ballistics regardless of horizontal robot speed.
            // =================================================================
            distanceM = MathUtil.clamp(hubPivot.getNorm(),
                    SuperstructureConstants.MIN_SHOOT_RANGE_M,
                    SuperstructureConstants.MAX_SHOOT_RANGE_M);
            ShooterSetpoint reqShot = ShooterKinematics.calculate(distanceM);

            // Decompose stationary-shot exit velocity into horizontal and vertical
            double v0      = ShooterKinematics.rpmToLaunchSpeed(reqShot.flywheelRPM());
            double exitRad = Math.toRadians(90.0 - reqShot.hoodAngleDeg());
            double vHoriz  = v0 * Math.cos(exitRad); // horizontal speed toward hub (ground frame)
            double vVert   = v0 * Math.sin(exitRad); // vertical component — stays fixed

            // Required ground-frame horizontal velocity directed at hub
            double thetaHub = hubPivot.getAngle().getRadians();
            double vReqX    = vHoriz * Math.cos(thetaHub);
            double vReqY    = vHoriz * Math.sin(thetaHub);

            // Subtract PIVOT velocity (not robot center) — ball launches from pivot,
            // so the velocity contribution to the ball is V_pivot, not V_robot_center.
            double vRelX = vReqX - pivotVel.getX();
            double vRelY = vReqY - pivotVel.getY();

            // Turret aim direction and new horizontal speed
            alphaFireRad   = Math.atan2(vRelY, vRelX);
            alphaFutureRad = alphaFireRad; // EVS has no separate future angle
            double vRelHoriz = Math.hypot(vRelX, vRelY);

            // Combine new horizontal speed with original vy0 → new RPM + hood angle
            // Hood angle changes to reflect adjusted horizontal speed, not the distance.
            double vExitNew   = Math.hypot(vRelHoriz, vVert);
            double exitRadNew = Math.atan2(vVert, vRelHoriz);
            double hoodDegNew = MathUtil.clamp(90.0 - Math.toDegrees(exitRadNew),
                    Shooter.HOOD_MIN_ANGLE_DEG, Shooter.HOOD_MAX_ANGLE_DEG);
            setpoint = new ShooterSetpoint(ShooterKinematics.v0ToRPM(vExitNew), hoodDegNew);

            // distanceM = actual hub distance (used for turret tracking rate below)
            distanceM = hubPivot.getNorm();
        }

        // vLateral: component of pivot velocity perpendicular to hub direction.
        // Used only for commandTurretAngle signature — NOT for modelAlphaDot.
        // Computed from hub direction (not fire angle) to avoid EVS circular dependency.
        double hubAngleRad = hubPivot.getAngle().getRadians();
        double vLateral = pivotVel.getX() * -Math.sin(hubAngleRad)
                        + pivotVel.getY() *  Math.cos(hubAngleRad);

        // =====================================================================
        // TURRET ANGULAR VELOCITY FEEDFORWARD
        //
        //   modelAlphaDot is the exact analytical derivative of the hub angle
        //   in robot frame, computed from hub coordinates — NOT from alphaFireRad.
        //   Using alphaFireRad (the EVS output) would create a circular dependency:
        //   omega changes alphaFireRad → alphaFireRad changes model → oscillation.
        //
        //   Derivation (hub fixed in ground frame, robot frame rotates):
        //     d/dt(alpha_hub) = −ω + (hub.y·vx − hub.x·vy) / |hub|²
        //
        //   measuredAlphaDot: numerical derivative of alphaFireRad per loop.
        //   Blended: more model weight at low speed (stable), more measured at
        //   high speed (captures EVS compensation angle rate).
        // =====================================================================

        double alphaDot;
        double hx = hubPivot.getX();
        double hy = hubPivot.getY();
        double hd2 = Math.max(hx * hx + hy * hy, 0.01);
        if (!m_alphaFireSeeded) {
            alphaDot           = 0.0;
            m_alphaDotFilt     = 0.0;
            m_lastAlphaFireRad = alphaFireRad;
            m_alphaFireSeeded  = true;
        } else {
            // Measured alphaDot: numerical derivative of the EVS fire angle.
            double rawAlphaDot = MathUtil.angleModulus(alphaFireRad - m_lastAlphaFireRad);
            double measuredAlphaDot = MathUtil.clamp(
                    rawAlphaDot / dt,
                    -Shooter.SOTM_MAX_ALPHA_DOT_RAD_PER_S,
                    Shooter.SOTM_MAX_ALPHA_DOT_RAD_PER_S);

            // Exact model: d/dt(alpha_hub) = −ω + cross(hub, vPivot) / |hub|²
            // cross(hub, v) = hub.y·vx − hub.x·vy  (2-D cross product, z-component)
            double modelAlphaDot = MathUtil.clamp(
                    -omega + (hy * pivotVel.getX() - hx * pivotVel.getY()) / hd2,
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

            if (!inShooting || m_isIntaking.getAsBoolean()) {
                // Not shooting yet — cancel any agitate state and deploy the arm
                // so it is ready when SHOOTING begins.
                if (m_agitating || m_holdingAgitate) {
                    m_intake.deploy();
                    m_intake.stopRoller();
                    m_agitating      = false;
                    m_holdingAgitate = false;
                }
            } else if (m_isIntaking.getAsBoolean()) {
                // Roller held — deploy the arm so intake is ready.
                // Do NOT touch the roller here; the roller command owns it.
                m_intake.deploy();
                m_agitating      = false;
                m_holdingAgitate = false;
                m_agitateIntervalTimer.restart();
            } else if (m_agitating) {
                // Active agitate pulse: hold position and run roller.
                m_intake.agitate();
                m_intake.runRollerAt(AGITATE_ROLLER_PERCENT);
                if (m_agitatePhaseTimer.hasElapsed(AGITATE_DURATION_S)) {
                    // Pulse done — stay at agitate position and keep roller running.
                    m_agitating      = false;
                    m_holdingAgitate = true;
                }
            } else if (m_holdingAgitate) {
                // Hold agitate position with roller running until operator runs rollers.
                m_intake.agitate();
                m_intake.runRollerAt(AGITATE_ROLLER_PERCENT);
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
