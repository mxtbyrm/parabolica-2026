package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
 * <p>The side is recomputed every loop so the target updates if the robot
 * crosses the centre line while the command is active.
 *
 * <p>Uses the same EVS (Exact Vector Subtraction) SOTM algorithm as
 * {@link ShootCommand}: vector subtraction of pivot velocity from required
 * ground-frame ball velocity, and hub-coordinate alphaDot feedforward.
 * No latency prediction — drivetrain pose is already fused from 4 PhotonVision
 * cameras + odometry, so every loop prepares setpoints as if a ball exits
 * right now (instantaneous SOTM).  The turret lead angle and RPM/hood setpoints
 * are adjusted for the robot's current velocity so passes are accurate whether
 * the robot is stationary or moving.
 *
 * <p>Hold the button to pass; releasing returns to {@link RobotState#STOWED}.
 */
public class PassCommand extends Command {

    private final Superstructure          m_superstructure;
    private final CommandSwerveDrivetrain m_drivetrain;

    /** Pass target selected once in initialize(); does not change mid-command. */
    private Translation2d m_passTarget;

    public PassCommand(Superstructure superstructure,
                       CommandSwerveDrivetrain drivetrain) {
        m_superstructure = superstructure;
        m_drivetrain     = drivetrain;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        m_passTarget = selectPassTarget();
        m_superstructure.requestState(RobotState.PREPPING_TO_PASS);
    }

    @Override
    public void execute() {
        // ── Chassis velocity — read once, used for readiness gate and EVS ────
        ChassisSpeeds rawSpd = m_drivetrain.getState().Speeds;
        double vx    = rawSpd.vxMetersPerSecond;
        double vy    = rawSpd.vyMetersPerSecond;
        double omega = rawSpd.omegaRadiansPerSecond;
        double chassisSpeedMps = Math.hypot(vx, vy);

        // Stationary thresholds match ShootCommand:
        //   isNearlyStationary → tight-tolerance readiness gate
        //   isStationary       → EVS deadband (wider: SOTM_SPEED_DEADBAND_MPS)
        boolean isNearlyStationary = chassisSpeedMps < 0.1 && Math.abs(omega) < 0.15;
        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS
                            && Math.abs(omega) < 0.05;

        RobotState cur = m_superstructure.getState();
        // Do not override INTAKE_UNSAFE, EXHAUSTING, or WRAPAROUND — state machine handles them.
        // WRAPAROUND must complete (turret slews to new position) before re-entering PREPPING_TO_PASS;
        // interrupting it every loop causes PREPPING_TO_PASS↔WRAPAROUND oscillation where the
        // turret never actually slews and the system never reaches PASSING_TO_ALLIANCE.
        if (cur != RobotState.PREPPING_TO_PASS
                && cur != RobotState.PASSING_TO_ALLIANCE
                && cur != RobotState.EXHAUSTING
                && cur != RobotState.WRAPAROUND
                && cur != RobotState.INTAKE_UNSAFE) {
            m_superstructure.requestState(RobotState.PREPPING_TO_PASS);
        }

        // Gate: advance to PASSING_TO_ALLIANCE once mechanisms are ready.
        // Stationary: require all three within tight static tolerances.
        // Moving: use wide tracking tolerances — SOTM shifts the turret setpoint
        // every loop so the tight ±1° gate is never satisfied during motion.
        if (cur == RobotState.PREPPING_TO_PASS) {
            boolean ready = isNearlyStationary
                    ? m_superstructure.isReadyToPass()
                    : m_superstructure.isTrackingSetpoints();
            if (ready) {
                m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
            }
        }

        // ── Target vector in robot frame ─────────────────────────────────────
        // Target was fixed at initialize() — robot won't cross the centre line
        // during a single pass action, so no need to recompute each loop.
        Translation2d passTarget = m_passTarget;
        Translation2d turretPos  = getTurretPivotPosition();
        var robotPose = m_drivetrain.getState().Pose;
        Translation2d delta      = passTarget.minus(turretPos);
        Translation2d deltaRobot = delta.rotateBy(robotPose.getRotation().unaryMinus());
        double targetDist  = delta.getNorm();
        double alphaNowRad = Math.atan2(deltaRobot.getY(), deltaRobot.getX());

        // Turret pivot velocity in robot frame (includes ω × offset cross-term)
        double vxT = vx - omega * Turret.TURRET_OFFSET_Y_M;
        double vyT = vy + omega * Turret.TURRET_OFFSET_X_M;

        ShooterSetpoint setpoint;
        double alphaFireRad;
        double distanceM;

        // Target is measured from turret pivot position — deltaRobot is already
        // pivot-relative, so no additional turret-offset subtraction is needed.
        Translation2d pivotVel = new Translation2d(vxT, vyT);

        if (isStationary) {
            // Stationary: no movement to correct — aim straight at target.
            distanceM    = targetDist;
            setpoint     = ShooterKinematics.calculatePass(distanceM);
            alphaFireRad = alphaNowRad;
        } else {
            // =================================================================
            // EVS (Exact Vector Subtraction)
            //
            //   Drivetrain pose is already fused — no latency window to predict.
            //   Required ground-frame horizontal velocity aimed at current target;
            //   subtract pivot velocity to get turret-relative velocity.
            //   Vertical component (vVert, hood angle) from stationary table →
            //   rim clearance unchanged regardless of robot speed.
            // =================================================================
            distanceM = MathUtil.clamp(targetDist,
                    SuperstructureConstants.MIN_SHOOT_RANGE_M,
                    SuperstructureConstants.MAX_PASS_RANGE_M);
            setpoint = ShooterKinematics.calculatePass(distanceM);

            double v0       = ShooterKinematics.rpmToLaunchSpeed(setpoint.flywheelRPM());
            double exitRad  = Math.toRadians(90.0 - setpoint.hoodAngleDeg());
            double vHoriz   = v0 * Math.cos(exitRad);
            double vVert    = v0 * Math.sin(exitRad); // keeps rim clearance intact

            double thetaHub = alphaNowRad;
            double vRelX    = vHoriz * Math.cos(thetaHub) - pivotVel.getX();
            double vRelY    = vHoriz * Math.sin(thetaHub) - pivotVel.getY();
            alphaFireRad    = Math.atan2(vRelY, vRelX);

            double vRelHoriz = Math.hypot(vRelX, vRelY);
            double vExitNew  = Math.hypot(vRelHoriz, vVert);
            double exitRadNew = Math.atan2(vVert, vRelHoriz);
            double hoodDegNew = MathUtil.clamp(90.0 - Math.toDegrees(exitRadNew),
                    Shooter.HOOD_MIN_ANGLE_DEG, Shooter.HOOD_MAX_ANGLE_DEG);
            setpoint  = new ShooterSetpoint(ShooterKinematics.v0ToRPM(vExitNew), hoodDegNew);
            distanceM = targetDist;
        }

        // ── Apply setpoints ───────────────────────────────────────────────────
        m_superstructure.applyShooterSetpoint(setpoint);
        m_superstructure.commandTurretAngle(Math.toDegrees(alphaFireRad));

        SmartDashboard.putNumber("Pass/DistanceM",    targetDist);
        SmartDashboard.putNumber("Pass/DFireM",       distanceM);
        SmartDashboard.putNumber("Pass/AlphaFireDeg", Math.toDegrees(alphaFireRad));
        SmartDashboard.putNumber("Pass/TargetX",      passTarget.getX());
        SmartDashboard.putNumber("Pass/TargetY",      passTarget.getY());
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
