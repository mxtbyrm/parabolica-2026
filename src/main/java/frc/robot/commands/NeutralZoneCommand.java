package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.FieldLayout;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Pathfinds the robot from the alliance zone through the nearest TRENCH to the
 * neutral zone at the field midpoint.
 *
 * <h2>Sequence</h2>
 * <ol>
 *   <li>Selects the TRENCH structure whose neutral-side exit pose is closest to
 *       the robot's current position.</li>
 *   <li>If the robot is on the alliance (hub) side of the selected TRENCH,
 *       pathfinds through the trench opening to the neutral-side exit pose,
 *       forcing the path through the opening rather than around it.</li>
 *   <li>Pathfinds to the neutral zone target pose at the field midpoint X,
 *       aligned with the chosen trench's Y center.</li>
 * </ol>
 *
 * <p>If the robot is already on the neutral side of the trench, the transit
 * step is skipped and the robot drives directly to the neutral zone pose.
 *
 * <h2>Trench Safety</h2>
 * <p>{@link frc.robot.subsystems.TrenchTraversalManager} runs independently and
 * automatically stows all overhead mechanisms when the robot is near or inside
 * the TRENCH.  This command does not need to manage mechanism state directly.
 * On command end the {@code TRAVERSING_TRENCH} state is cleared if it is still
 * active.
 *
 * <h2>Target Poses (per trench)</h2>
 * <ul>
 *   <li>Blue: {@link FieldLayout#BLUE_NEUTRAL_ZONE_POSES} — field midpoint X,
 *       trench Y, facing 0°.</li>
 *   <li>Red:  {@link FieldLayout#RED_NEUTRAL_ZONE_POSES}  — field midpoint X,
 *       trench Y, facing 180°.</li>
 * </ul>
 */
public class NeutralZoneCommand {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        3.0,                             // max velocity m/s
        2.5,                             // max acceleration m/s²
        Units.degreesToRadians(540),     // max angular velocity rad/s
        Units.degreesToRadians(720)      // max angular acceleration rad/s²
    );

    private NeutralZoneCommand() {}

    /**
     * Creates the neutral-zone command.
     *
     * <p>Alliance color and trench selection are evaluated lazily at schedule
     * time via {@link Commands#defer}, so the correct poses are used even if
     * the alliance is not yet known at construction.
     *
     * @param drivetrain     The swerve drivetrain subsystem.
     * @param superstructure The superstructure state machine (used only for
     *                       state cleanup on command end).
     * @return A command that transits the nearest TRENCH and pathfinds to the
     *         neutral zone at the field midpoint.
     */
    public static Command create(CommandSwerveDrivetrain drivetrain,
                                 Superstructure superstructure) {
        return Commands.defer(
            () -> {
                boolean isRed = DriverStation.getAlliance()
                        .map(a -> a == DriverStation.Alliance.Red)
                        .orElse(false);

                Pose2d currentPose = drivetrain.getState().Pose;

                Pose2d[] neutralThroughPoses = isRed
                        ? FieldLayout.RED_TRENCH_NEUTRAL_THROUGH_POSES
                        : FieldLayout.BLUE_TRENCH_NEUTRAL_THROUGH_POSES;

                Pose2d[] neutralZonePoses = isRed
                        ? FieldLayout.RED_NEUTRAL_ZONE_POSES
                        : FieldLayout.BLUE_NEUTRAL_ZONE_POSES;

                // Pick the TRENCH whose neutral-side exit pose is closest to the robot.
                int closestIdx = 0;
                double minDist = currentPose.getTranslation()
                        .getDistance(neutralThroughPoses[0].getTranslation());
                for (int i = 1; i < neutralThroughPoses.length; i++) {
                    double d = currentPose.getTranslation()
                            .getDistance(neutralThroughPoses[i].getTranslation());
                    if (d < minDist) {
                        minDist = d;
                        closestIdx = i;
                    }
                }

                Pose2d throughPose   = neutralThroughPoses[closestIdx];
                Pose2d neutralTarget = neutralZonePoses[closestIdx];

                // Robot needs to transit the trench if it is still on the alliance
                // (hub) side — i.e. it has not yet passed the neutral-side exit pose.
                // Blue hub is at low X: robot is on hub side if X < throughPose X.
                // Red  hub is at high X: robot is on hub side if X > throughPose X.
                boolean needsTrenchTransit = isRed
                        ? currentPose.getX() > throughPose.getX()
                        : currentPose.getX() < throughPose.getX();

                if (needsTrenchTransit) {
                    return Commands.sequence(
                        AutoBuilder.pathfindToPose(throughPose, CONSTRAINTS),
                        AutoBuilder.pathfindToPose(neutralTarget, CONSTRAINTS));
                } else {
                    return AutoBuilder.pathfindToPose(neutralTarget, CONSTRAINTS);
                }
            },
            Set.of(drivetrain)
        ).finallyDo(interrupted -> {
            // Clear TRAVERSING_TRENCH that TrenchTraversalManager may have set
            // while the robot was inside the trench zone.
            if (superstructure.getState() == RobotState.TRAVERSING_TRENCH) {
                superstructure.requestState(RobotState.STOWED);
            }
        });
    }
}
