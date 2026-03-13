package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;
import frc.robot.util.ShooterKinematics;

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
 * <p>Hold the button to pass; releasing returns to {@link RobotState#STOWED}.
 */
public class PassCommand extends Command {

    private final Superstructure          m_superstructure;
    private final CommandSwerveDrivetrain m_drivetrain;

    /** Pass target latched on initialize() — does not change mid-command. */
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
        m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
    }

    @Override
    public void execute() {
        RobotState cur = m_superstructure.getState();
        if (cur != RobotState.PASSING_TO_ALLIANCE
                && cur != RobotState.EXHAUSTING) {
            m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
        }

        Translation2d turretPos = getTurretPivotPosition();
        double dist = clampRange(turretPos.getDistance(m_passTarget));

        m_superstructure.applyShooterSetpoint(ShooterKinematics.calculate(dist));
        commandTurretToTarget(turretPos, m_passTarget);

        SmartDashboard.putNumber("Pass/DistanceM", dist);
        SmartDashboard.putNumber("Pass/TargetX",   m_passTarget.getX());
        SmartDashboard.putNumber("Pass/TargetY",   m_passTarget.getY());
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

    /** Clamps distance to the shootable range. */
    private double clampRange(double dist) {
        return Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
               Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, dist));
    }

    /**
     * Commands the turret to face the pass target.
     * Converts the field-relative angle to the target into a robot-relative
     * turret angle by subtracting the robot's current heading.
     */
    private void commandTurretToTarget(Translation2d turretPos, Translation2d target) {
        Translation2d delta = target.minus(turretPos);
        double fieldAngleDeg = Math.toDegrees(Math.atan2(delta.getY(), delta.getX()));
        double robotHeadingDeg = m_drivetrain.getState().Pose.getRotation().getDegrees();
        m_superstructure.commandTurretAngle(fieldAngleDeg - robotHeadingDeg);
    }
}
