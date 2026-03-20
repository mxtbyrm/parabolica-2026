package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;
import frc.robot.util.ShooterKinematics;
import frc.robot.util.ShooterKinematics.ShooterSetpoint;

/**
 * Passes balls toward a specific field position where an alliance partner can
 * collect them.  The target is chosen based on which side of the hub the robot
 * is currently on (split at the field Y-centre = FIELD_WIDTH_M / 2):
 *
 * <ul>
 *   <li><b>Right side</b> (robot Y &lt; field centre): midpoint of the bottom wall
 *       and hub right face (Y), midpoint of the alliance wall and hub front face (X).</li>
 *   <li><b>Left side</b>  (robot Y &ge; field centre): midpoint of hub left face and
 *       top wall (Y), midpoint of the alliance wall and hub geometric centre (X).</li>
 * </ul>
 *
 * <p>The side is latched at command {@link #initialize()} so the target does not
 * jump if the robot crosses the centre line during the pass.
 *
 * <p>Uses the same EVS (Exact Vector Subtraction) SOTM algorithm as
 * {@link ShootCommand}: latency compensation, vector subtraction of pivot
 * velocity from required ground-frame ball velocity, and hub-coordinate
 * alphaDot feedforward.  The turret lead angle and RPM/hood setpoints are
 * adjusted for the robot's current velocity so passes are accurate whether
 * the robot is stationary or moving.
 *
 * <p>Hold the button to pass; releasing returns to {@link RobotState#STOWED}.
 */
public class PassCommand extends Command {

    private final Superstructure          m_superstructure;
    private final CommandSwerveDrivetrain m_drivetrain;

    /** Pass target latched on initialize() — does not change mid-command. */
    private Translation2d m_passTarget;

    // EMA for chassis velocity (same filter as ShootCommand)
    private double  m_filtVx    = 0.0;
    private double  m_filtVy    = 0.0;
    private double  m_filtOmega = 0.0;
    private boolean m_velSeeded = false;

    // Turret angular velocity feedforward — tracks lead-angle rate of change
    private double  m_lastAlphaFireRad  = 0.0;
    private boolean m_alphaFireSeeded   = false;
    private double  m_alphaDotFilt      = 0.0;

    // Real dt tracking — avoids fixed 0.020 assumption under scheduler jitter
    private double  m_lastTimestamp     = 0.0;

    public PassCommand(Superstructure superstructure,
                       CommandSwerveDrivetrain drivetrain) {
        m_superstructure = superstructure;
        m_drivetrain     = drivetrain;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        m_passTarget      = selectPassTarget();
        m_velSeeded       = false;
        m_alphaFireSeeded = false;
        m_alphaDotFilt    = 0.0;
        m_lastTimestamp   = Timer.getFPGATimestamp();
        m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
    }

    @Override
    public void execute() {
        RobotState cur = m_superstructure.getState();
        // Do not override INTAKE_UNSAFE — state machine handles it automatically.
        if (cur != RobotState.PASSING_TO_ALLIANCE
                && cur != RobotState.EXHAUSTING
                && cur != RobotState.INTAKE_UNSAFE) {
            m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
        }

        // ── Target vector in robot frame ─────────────────────────────────────
        Translation2d turretPos = getTurretPivotPosition();
        var robotPose = m_drivetrain.getState().Pose;
        Translation2d delta      = m_passTarget.minus(turretPos);
        Translation2d deltaRobot = delta.rotateBy(robotPose.getRotation().unaryMinus());
        double targetDist  = delta.getNorm();
        double alphaNowRad = Math.atan2(deltaRobot.getY(), deltaRobot.getX());

        // ── EMA velocity ──────────────────────────────────────────────────────
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
        double vx    = m_filtVx;
        double vy    = m_filtVy;
        double omega = m_filtOmega;
        double chassisSpeedMps = Math.hypot(vx, vy);

        // ── Real dt (FPGA-accurate; guards against scheduler jitter) ─────────
        double now = Timer.getFPGATimestamp();
        double dt  = MathUtil.clamp(now - m_lastTimestamp, 0.005, 0.040);
        m_lastTimestamp = now;

        // Turret pivot velocity in robot frame (includes ω × offset cross-term)
        double vxT = vx - omega * Turret.TURRET_OFFSET_Y_M;
        double vyT = vy + omega * Turret.TURRET_OFFSET_X_M;

        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS
                            && Math.abs(omega) < 0.05;

        ShooterSetpoint setpoint;
        double alphaFireRad;
        double distanceM;

        // Target is measured from turret pivot position — deltaRobot is already
        // pivot-relative, so no additional turret-offset subtraction is needed.
        Translation2d pivotVel = new Translation2d(vxT, vyT);
        Translation2d robotVel = new Translation2d(vx, vy);

        // hubPivot: latency-compensated target vector in robot/pivot frame.
        // Hoisted outside if-else so alphaDot can always reference it.
        Translation2d hubPivot;

        if (isStationary) {
            // Stationary: no movement to correct — aim straight at target.
            distanceM    = targetDist;
            setpoint     = ShooterKinematics.calculatePass(distanceM);
            alphaFireRad = alphaNowRad;
            hubPivot     = new Translation2d(targetDist, new Rotation2d(alphaNowRad));
        } else {
            // =================================================================
            // STEP 1 — LATENCY COMPENSATION
            //
            //   Compensate for robot motion during the feeder+flywheel pipeline
            //   latency (SOTM_LATENCY_S).  robotVel used for translation;
            //   omega for frame rotation over the latency window.
            // =================================================================
            hubPivot = new Translation2d(targetDist, new Rotation2d(alphaNowRad))
                    .rotateBy(new Rotation2d(-omega * Shooter.SOTM_LATENCY_S))
                    .minus(robotVel.times(Shooter.SOTM_LATENCY_S));

            // =================================================================
            // STEP 2 — EVS (Exact Vector Subtraction)
            //
            //   Required ground-frame ball velocity = stationary-shot velocity
            //   aimed at hubPivot.  Subtract pivot velocity to get turret-relative
            //   velocity.  vy0 (vertical component) is kept from stationary shot
            //   at the actual distance → rim clearance is preserved.
            //   No tof iteration needed; single lookup is exact.
            // =================================================================
            distanceM = MathUtil.clamp(hubPivot.getNorm(),
                    SuperstructureConstants.MIN_SHOOT_RANGE_M,
                    SuperstructureConstants.MAX_SHOOT_RANGE_M);
            setpoint = ShooterKinematics.calculatePass(distanceM);

            double v0       = ShooterKinematics.rpmToLaunchSpeed(setpoint.flywheelRPM());
            double exitRad  = Math.toRadians(90.0 - setpoint.hoodAngleDeg());
            double vHoriz   = v0 * Math.cos(exitRad);
            double vVert    = v0 * Math.sin(exitRad); // keeps rim clearance intact

            double thetaHub = hubPivot.getAngle().getRadians();
            double vRelX    = vHoriz * Math.cos(thetaHub) - pivotVel.getX();
            double vRelY    = vHoriz * Math.sin(thetaHub) - pivotVel.getY();
            alphaFireRad    = Math.atan2(vRelY, vRelX);

            double vRelHoriz = Math.hypot(vRelX, vRelY);
            double vExitNew  = Math.hypot(vRelHoriz, vVert);
            double exitRadNew = Math.atan2(vVert, vRelHoriz);
            double hoodDegNew = MathUtil.clamp(90.0 - Math.toDegrees(exitRadNew),
                    Shooter.HOOD_MIN_ANGLE_DEG, Shooter.HOOD_MAX_ANGLE_DEG);
            setpoint  = new ShooterSetpoint(ShooterKinematics.v0ToRPM(vExitNew), hoodDegNew);
            distanceM = hubPivot.getNorm();
        }

        // vLateral: pivot-frame lateral velocity perpendicular to hub direction.
        // Uses hubPivot direction (not alphaFireRad) to avoid circular dependency.
        double hubAngleRad = hubPivot.getAngle().getRadians();
        double vLateral = pivotVel.getX() * -Math.sin(hubAngleRad)
                        + pivotVel.getY() *  Math.cos(hubAngleRad);

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
            double rawAlphaDot = MathUtil.angleModulus(alphaFireRad - m_lastAlphaFireRad);
            double measuredAlphaDot = MathUtil.clamp(
                    rawAlphaDot / dt,
                    -Shooter.SOTM_MAX_ALPHA_DOT_RAD_PER_S,
                    Shooter.SOTM_MAX_ALPHA_DOT_RAD_PER_S);

            // Exact hub-coordinate derivative: d/dt(alpha) = -omega + cross(hub, v) / |hub|²
            double hx  = hubPivot.getX(), hy = hubPivot.getY();
            double hd2 = Math.max(hx * hx + hy * hy, 0.01);
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

        // ── Apply setpoints ───────────────────────────────────────────────────
        m_superstructure.applyShooterSetpoint(setpoint);
        m_superstructure.commandTurretAngle(Math.toDegrees(alphaFireRad), vLateral, distanceM,
                                            alphaDot);

        SmartDashboard.putNumber("Pass/DistanceM",    targetDist);
        SmartDashboard.putNumber("Pass/DFireM",       distanceM);
        SmartDashboard.putNumber("Pass/AlphaFireDeg", Math.toDegrees(alphaFireRad));
        SmartDashboard.putNumber("Pass/AlphaDotRadPerSec", alphaDot);
        SmartDashboard.putNumber("Pass/TargetX",      m_passTarget.getX());
        SmartDashboard.putNumber("Pass/TargetY",      m_passTarget.getY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_superstructure.requestState(RobotState.STOWED);
    }

    // =========================================================================
    // Private Helpers
    // =========================================================================

    /**
     * Picks the pass target based on the robot's current Y position and alliance.
     * Right side = Y < FIELD_WIDTH_M/2 (bottom/right wall side).
     * Left side  = Y >= FIELD_WIDTH_M/2 (top/left wall side).
     */
    private Translation2d selectPassTarget() {
        boolean isRed = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        double robotY = m_drivetrain.getState().Pose.getTranslation().getY();
        boolean isRightSide = robotY < FieldLayout.FIELD_WIDTH_M / 2.0;

        if (isRed) {
            return isRightSide ? FieldLayout.RED_PASS_TARGET_RIGHT : FieldLayout.RED_PASS_TARGET_LEFT;
        } else {
            return isRightSide ? FieldLayout.BLUE_PASS_TARGET_RIGHT : FieldLayout.BLUE_PASS_TARGET_LEFT;
        }
    }

    /** Returns the turret pivot position in field coordinates. */
    private Translation2d getTurretPivotPosition() {
        var robotPose = m_drivetrain.getState().Pose;
        return robotPose.getTranslation().plus(
                new Translation2d(Turret.TURRET_OFFSET_X_M, Turret.TURRET_OFFSET_Y_M)
                        .rotateBy(robotPose.getRotation()));
    }
}
