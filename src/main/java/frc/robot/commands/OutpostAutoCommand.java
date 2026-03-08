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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;

/**
 * Autonomous routine that pathfinds to the alliance OUTPOST while shooting
 * preloaded balls, collects balls from the HUMAN PLAYER, then shoots them.
 *
 * <h2>Sequence</h2>
 * <ol>
 *   <li><b>Phase 1 — Drive &amp; Shoot:</b> Pathfind to the OUTPOST approach
 *       pose while simultaneously shooting all preloaded balls at the HUB.
 *       The turret tracks the HUB with full SOTM compensation during the drive.</li>
 *   <li><b>Phase 2 — Collect:</b> Deploy the intake and run the roller for
 *       {@link FieldLayout#OUTPOST_COLLECT_TIME_S} seconds (default 10 s).
 *       The HUMAN PLAYER opens the CHUTE DOOR and balls roll down into
 *       the CORRAL where the intake collects them.</li>
 *   <li><b>Phase 3 — Shoot collected:</b> Shoot the newly collected balls
 *       at the HUB with a 5-second timeout.</li>
 * </ol>
 *
 * <p>Alliance color is evaluated lazily at schedule time via
 * {@link Commands#defer} so the correct OUTPOST pose is selected.
 */
public class OutpostAutoCommand {

    /** PathPlanner constraints for the OUTPOST approach path. */
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        3.0,                             // max velocity m/s
        2.5,                             // max acceleration m/s²
        Units.degreesToRadians(540),     // max angular velocity rad/s
        Units.degreesToRadians(720)      // max angular acceleration rad/s²
    );

    /** Timeout for the drive+shoot phase so auto doesn't stall indefinitely. */
    private static final double DRIVE_SHOOT_TIMEOUT_S = 15.0;

    /** Timeout for the final shoot phase after collecting balls. */
    private static final double FINAL_SHOOT_TIMEOUT_S = 5.0;

    private OutpostAutoCommand() {}

    /**
     * Creates the OUTPOST auto command.
     *
     * @param drivetrain     The swerve drivetrain subsystem.
     * @param superstructure The superstructure state machine.
     * @param vision         The Limelight vision subsystem.
     * @param photonVision   The PhotonVision subsystem.
     * @param intake         The intake subsystem.
     * @return A deferred command implementing the full OUTPOST auto routine.
     */
    public static Command create(CommandSwerveDrivetrain drivetrain,
                                 Superstructure superstructure,
                                 VisionSubsystem vision,
                                 PhotonVisionSubsystem photonVision,
                                 IntakeSubsystem intake) {
        return Commands.defer(
            () -> {
                boolean isRed = DriverStation.getAlliance()
                        .map(a -> a == DriverStation.Alliance.Red)
                        .orElse(false);

                Pose2d outpostPose = isRed
                        ? FieldLayout.RED_OUTPOST_APPROACH_POSE
                        : FieldLayout.BLUE_OUTPOST_APPROACH_POSE;

                return Commands.sequence(
                    // Phase 1: Pathfind to OUTPOST while shooting preloaded balls.
                    // The pathfind command is the deadline — when the robot arrives,
                    // the parallel shoot command is interrupted and the feeder stops.
                    Commands.deadline(
                        AutoBuilder.pathfindToPose(outpostPose, CONSTRAINTS),
                        new AutoShootCommand(superstructure, vision, drivetrain,
                                             photonVision, DRIVE_SHOOT_TIMEOUT_S)
                    ),

                    // Phase 2: Collect balls from the OUTPOST CHUTE.
                    // Intake deploys, roller runs, then stows when the timeout expires.
                    new IntakeCommand(intake)
                            .withTimeout(FieldLayout.OUTPOST_COLLECT_TIME_S),

                    // Phase 3: Shoot the collected balls at the HUB.
                    new AutoShootCommand(superstructure, vision, drivetrain,
                                         photonVision, FINAL_SHOOT_TIMEOUT_S)
                );
            },
            Set.of(drivetrain, superstructure, vision, intake)
        );
    }
}
