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
 * Passes balls toward the robot's alliance wall using the same physics-based
 * kinematics as hub shooting.  The turret aims at the alliance wall while the
 * flywheel RPM and hood angle are computed from the distance to the wall.
 *
 * <p>This command is independent of hub state — the operator decides when to
 * pass rather than having it automatically triggered by the hub being inactive.
 *
 * <p>Hold the button to pass; releasing returns to {@link RobotState#STOWED}.
 */
public class PassCommand extends Command {

    private final Superstructure          m_superstructure;
    private final CommandSwerveDrivetrain m_drivetrain;

    public PassCommand(Superstructure superstructure,
                       CommandSwerveDrivetrain drivetrain) {
        m_superstructure = superstructure;
        m_drivetrain     = drivetrain;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
    }

    @Override
    public void execute() {
        RobotState cur = m_superstructure.getState();
        if (cur != RobotState.PASSING_TO_ALLIANCE
                && cur != RobotState.EXHAUSTING) {
            m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
        }

        double distToWall = computeDistanceToAllianceWall();
        m_superstructure.applyShooterSetpoint(ShooterKinematics.calculate(distToWall));
        commandTurretToAllianceWall();

        SmartDashboard.putNumber("Pass/DistanceM", distToWall);
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

    private double computeDistanceToAllianceWall() {
        var robotPose = m_drivetrain.getState().Pose;
        Translation2d turretPos = robotPose.getTranslation().plus(
                new Translation2d(Turret.TURRET_OFFSET_X_M, Turret.TURRET_OFFSET_Y_M)
                        .rotateBy(robotPose.getRotation()));
        boolean isRed = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        double wallX = isRed ? FieldLayout.FIELD_LENGTH_M : 0.0;
        double dist  = Math.abs(turretPos.getX() - wallX);
        return Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
               Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, dist));
    }

    private void commandTurretToAllianceWall() {
        double robotHeadingDeg = m_drivetrain.getState().Pose.getRotation().getDegrees();
        double wallFieldAngleDeg =
                (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                        == DriverStation.Alliance.Red)
                ? 0.0 : 180.0;
        m_superstructure.commandTurretAngle(wallFieldAngleDeg - robotHeadingDeg);
    }
}
