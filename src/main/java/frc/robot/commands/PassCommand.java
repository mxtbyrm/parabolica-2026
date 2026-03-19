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
 * <p>Uses the same SOTM algorithm as {@link ShootCommand}: latency compensation,
 * EMA-filtered TOF, iterative future-hub solve, and alphaDot feedforward.  The
 * turret lead angle and RPM/hood setpoints are adjusted for the robot's current
 * velocity so passes are accurate whether the robot is stationary or moving.
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
    private double  m_tofFilt           = 0.0;

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
        m_tofFilt         = 0.0;
        m_lastTimestamp   = Timer.getFPGATimestamp();
        m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
    }

    @Override
    public void execute() {
        RobotState cur = m_superstructure.getState();
        if (cur != RobotState.PASSING_TO_ALLIANCE
                && cur != RobotState.EXHAUSTING) {
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

        // Turret pivot offset in robot frame — target distance is measured from
        // turret pivot position, so no additional correction needed here.
        Translation2d pivotVel = new Translation2d(vxT, vyT);
        Translation2d robotVel = new Translation2d(vx, vy);

        if (isStationary) {
            // Stationary: skip latency/future correction — no movement to predict.
            distanceM    = targetDist;
            setpoint     = ShooterKinematics.calculatePass(distanceM);
            alphaFireRad = alphaNowRad;
        } else {
            // =================================================================
            // STEP 1 — LATENCY COMPENSATION
            //
            //   The target is computed from odometry, not vision, so there is no
            //   vision latency per se.  However the feeder+flywheel pipeline has
            //   its own latency (SOTM_LATENCY_S).  Compensate for robot motion
            //   during that window using robotVel (robot-center velocity).
            // =================================================================
            Translation2d hub = new Translation2d(targetDist, new Rotation2d(alphaNowRad))
                    .rotateBy(new Rotation2d(-omega * Shooter.SOTM_LATENCY_S))
                    .minus(robotVel.times(Shooter.SOTM_LATENCY_S));

            // =================================================================
            // STEP 2 — TOF FILTER (once, before iterations)
            //
            //   Filter is applied exactly once per execute() loop so the EMA
            //   alpha is correct and the tof used in both iterations is identical.
            // =================================================================
            distanceM = hub.getNorm();
            setpoint  = ShooterKinematics.calculatePass(distanceM);
            double clampedD0 = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                               Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, distanceM));
            double rawTof = ShooterKinematics.getFlightTimeSeconds(clampedD0, setpoint);
            double resetThreshold = MathUtil.clamp(0.1 + 0.02 * distanceM, 0.1, 0.25);
            if (m_tofFilt == 0.0 || Math.abs(rawTof - m_tofFilt) > resetThreshold) {
                m_tofFilt = rawTof;
            } else {
                double tofAlpha = MathUtil.clamp(0.45 - 0.04 * distanceM, 0.2, 0.45);
                m_tofFilt += tofAlpha * (rawTof - m_tofFilt);
            }
            double tof = m_tofFilt;

            // =================================================================
            // STEP 3 — ITERATIVE FUTURE HUB SOLVE (2 iterations)
            //
            //   tof is seeded from the EMA filter but IS updated inside the loop
            //   so each iteration uses a tof consistent with the refined distance.
            //   The filter state is NOT touched here.
            // =================================================================
            Translation2d futHub = hub;
            for (int i = 0; i < 2; i++) {
                futHub    = hub.minus(pivotVel.times(tof));
                distanceM = futHub.getNorm();
                setpoint  = ShooterKinematics.calculatePass(distanceM);
                double clampedD = MathUtil.clamp(distanceM,
                        SuperstructureConstants.MIN_SHOOT_RANGE_M,
                        SuperstructureConstants.MAX_SHOOT_RANGE_M);
                tof = ShooterKinematics.getFlightTimeSeconds(clampedD, setpoint);
            }

            // =================================================================
            // STEP 4 — VIRTUAL GOAL: aim at future hub, use future distance.
            //
            //   SOTM_LEAD_ANGLE_SCALAR scales the lateral lead angle so operators
            //   can correct persistent left/right error without recompiling.
            //   SOTM_RADIAL_SCALE scales the radial distance correction so
            //   operators can correct persistent over/under-range error.
            //   Both default to 1.0 (no change from the raw SOTM output).
            // =================================================================
            double hubAnglePivotRad = hub.getAngle().getRadians();
            double leadDelta        = MathUtil.angleModulus(futHub.getAngle().getRadians() - hubAnglePivotRad);
            alphaFireRad = MathUtil.angleModulus(hubAnglePivotRad
                    + leadDelta * Shooter.SOTM_LEAD_ANGLE_SCALAR);
            distanceM = hub.getNorm() + (distanceM - hub.getNorm()) * Shooter.SOTM_RADIAL_SCALE;
            setpoint  = ShooterKinematics.calculatePass(MathUtil.clamp(distanceM,
                    SuperstructureConstants.MIN_SHOOT_RANGE_M,
                    SuperstructureConstants.MAX_SHOOT_RANGE_M));
        }

        // vLateral: pivot-frame lateral velocity perpendicular to target direction.
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
            double rawAlphaDot = MathUtil.angleModulus(alphaFireRad - m_lastAlphaFireRad);
            double measuredAlphaDot = MathUtil.clamp(
                    rawAlphaDot / dt,
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
