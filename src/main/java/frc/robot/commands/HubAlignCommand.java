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
 * Pathfinds the robot through the nearest alliance TRENCH and on to the hub
 * approach pose, ready for a vision-guided shot.
 *
 * <h2>Sequence</h2>
 * <ol>
 *   <li>Immediately requests {@link RobotState#TRAVERSING_TRENCH} to stow all
 *       overhead mechanisms before the robot enters the TRENCH.</li>
 *   <li>Pathfinds to the hub-side exit of the closest TRENCH structure, forcing
 *       the path to route <em>through</em> the trench opening rather than around it.
 *       This step is skipped if the robot is already on the hub side of the
 *       trench (to avoid driving away from the hub).</li>
 *   <li>Pathfinds to the alliance hub approach pose.</li>
 * </ol>
 *
 * <h2>Target Poses</h2>
 * <ul>
 *   <li>Blue: {@link FieldLayout#BLUE_HUB_APPROACH_POSE} — 3.5 m in front of the
 *       Blue HUB, facing the HUB (0°).</li>
 *   <li>Red:  {@link FieldLayout#RED_HUB_APPROACH_POSE}  — 3.5 m in front of the
 *       Red HUB, facing the HUB (180°).</li>
 * </ul>
 *
 * <h2>Constraints</h2>
 * <ul>
 *   <li>Max velocity: 3.0 m/s</li>
 *   <li>Max acceleration: 2.5 m/s²</li>
 *   <li>Max angular velocity: 540°/s</li>
 *   <li>Max angular acceleration: 720°/s²</li>
 * </ul>
 */
public class HubAlignCommand {

    // PathPlanner constraints for the hub-approach pathfinding.
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        3.0,                             // max velocity m/s
        2.5,                             // max acceleration m/s²
        Units.degreesToRadians(540),     // max angular velocity rad/s
        Units.degreesToRadians(720)      // max angular acceleration rad/s²
    );

    // Private constructor — use the static factory method.
    private HubAlignCommand() {}

    /**
     * Creates the hub-align command for use in button bindings.
     *
     * <p>Alliance color and trench selection are evaluated lazily at schedule
     * time via {@link Commands#defer}, so the correct poses are chosen even if
     * the robot switches sides mid-match.
     *
     * @param drivetrain     The swerve drivetrain subsystem.
     * @param superstructure The superstructure state machine (stowed for trench transit).
     * @return A deferred command that stows mechanisms, transits the trench, and
     *         pathfinds to the alliance HUB approach pose.
     */
    public static Command create(CommandSwerveDrivetrain drivetrain,
                                 Superstructure superstructure) {
        return Commands.defer(
            () -> {
                boolean isRed = DriverStation.getAlliance()
                        .map(a -> a == DriverStation.Alliance.Red)
                        .orElse(false);

                Pose2d currentPose = drivetrain.getState().Pose;

                // Pick the TRENCH through-pose closest to the robot's current position.
                Pose2d[] trenchPoses = isRed
                        ? FieldLayout.RED_TRENCH_THROUGH_POSES
                        : FieldLayout.BLUE_TRENCH_THROUGH_POSES;

                Pose2d hubApproach = isRed
                        ? FieldLayout.RED_HUB_APPROACH_POSE
                        : FieldLayout.BLUE_HUB_APPROACH_POSE;

                Command stow = Commands.runOnce(
                        () -> superstructure.requestState(RobotState.TRAVERSING_TRENCH),
                        superstructure);

                // If no trench through-poses are configured, skip the transit step.
                if (trenchPoses.length == 0) {
                    return Commands.sequence(
                        stow,
                        AutoBuilder.pathfindToPose(hubApproach, CONSTRAINTS));
                }

                // Pick the TRENCH through-pose closest to the robot's current position.
                Pose2d closestTrench = trenchPoses[0];
                double minDist = currentPose.getTranslation()
                        .getDistance(trenchPoses[0].getTranslation());
                for (int i = 1; i < trenchPoses.length; i++) {
                    double d = currentPose.getTranslation()
                            .getDistance(trenchPoses[i].getTranslation());
                    if (d < minDist) {
                        minDist = d;
                        closestTrench = trenchPoses[i];
                    }
                }

                // Skip the trench via-pose if the robot is already on the hub side
                // (i.e. it has already passed through the trench), to avoid sending
                // it back in the wrong direction.
                boolean needsTrenchTransit = isRed
                        ? currentPose.getX() < closestTrench.getX()
                        : currentPose.getX() > closestTrench.getX();

                if (needsTrenchTransit) {
                    return Commands.sequence(
                        stow,
                        AutoBuilder.pathfindToPose(closestTrench, CONSTRAINTS),
                        AutoBuilder.pathfindToPose(hubApproach, CONSTRAINTS));
                } else {
                    return Commands.sequence(
                        stow,
                        AutoBuilder.pathfindToPose(hubApproach, CONSTRAINTS));
                }
            },
            Set.of(drivetrain, superstructure)
        );
    }
}
