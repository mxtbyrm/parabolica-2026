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
import frc.robot.Constants.OutpostConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Full OUTPOST autonomous routine.
 *
 * <h2>Sequence</h2>
 * <ol>
 *   <li><b>Pose init</b> — Waits up to 2 s for PhotonVision to deliver at least
 *       one corrected pose estimate before starting the path, so PathPlanner
 *       starts from an accurate location.</li>
 *   <li><b>Drive + shoot</b> — Pathfinds to the alliance OUTPOST dock pose while
 *       concurrently running {@link AutoShootCommand} to score any pre-loaded
 *       FUEL.  This phase ends as soon as the robot reaches the dock pose (the
 *       shoot command is cancelled at that point even if balls remain).</li>
 *   <li><b>Wait at OUTPOST</b> — Robot holds position for
 *       {@link OutpostConstants#OUTPOST_WAIT_SECONDS} (10 s) while the HUMAN
 *       PLAYER opens the CHUTE DOOR and the full chute capacity of FUEL rolls
 *       into the robot.</li>
 *   <li><b>Shoot received FUEL</b> — Sets ball count to
 *       {@link OutpostConstants#OUTPOST_CHUTE_CAPACITY} and runs
 *       {@link AutoShootCommand} until all balls are scored or the timeout
 *       expires.</li>
 * </ol>
 *
 * <h2>Target poses</h2>
 * <ul>
 *   <li>Blue: {@link FieldLayout#BLUE_OUTPOST_DOCK_POSE} — 0.5 m from Blue wall,
 *       Y = OUTPOST center, robot faces 180° (toward Blue wall).</li>
 *   <li>Red:  {@link FieldLayout#RED_OUTPOST_DOCK_POSE}  — symmetric, faces 0°.</li>
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
public class OutpostAutoCommand {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        3.0,
        2.5,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    // Private constructor — static factory only.
    private OutpostAutoCommand() {}

    /**
     * Creates the outpost auto command.
     *
     * <p>Alliance color is evaluated lazily at schedule time via
     * {@link Commands#defer}, so this factory can be called during construction
     * before the DS has reported the alliance.
     *
     * @param drivetrain   The swerve drivetrain subsystem.
     * @param superstructure The superstructure state machine.
     * @param vision       The Limelight vision subsystem.
     * @param photonVision The PhotonVision subsystem (pose correction + hub angle).
     * @return The full outpost auto sequence.
     */
    public static Command create(
            CommandSwerveDrivetrain drivetrain,
            Superstructure superstructure,
            VisionSubsystem vision,
            PhotonVisionSubsystem photonVision) {

        return Commands.defer(() -> {
            boolean isRed = DriverStation.getAlliance()
                    .map(a -> a == DriverStation.Alliance.Red)
                    .orElse(false);

            Pose2d outpostPose = isRed
                    ? FieldLayout.RED_OUTPOST_DOCK_POSE
                    : FieldLayout.BLUE_OUTPOST_DOCK_POSE;

            // Phase 1 drive: pathfind to the outpost dock pose.
            Command driveToOutpost = AutoBuilder.pathfindToPose(outpostPose, CONSTRAINTS);

            // Phase 1 shoot: score pre-loaded balls while driving.
            // ShootCommand does not require the drivetrain, so these run in parallel.
            Command shootWhileDriving = new AutoShootCommand(
                    superstructure, vision, drivetrain, photonVision,
                    OutpostConstants.OUTPOST_DRIVE_SHOOT_TIMEOUT_S);

            // Phase 2: shoot all balls received from the OUTPOST.
            Command shootReceived = new AutoShootCommand(
                    superstructure, vision, drivetrain, photonVision,
                    OutpostConstants.OUTPOST_RECEIVE_SHOOT_TIMEOUT_S);

            return Commands.sequence(
                // Wait for PhotonVision pose correction before trusting odometry.
                Commands.waitUntil(photonVision::hasPoseBeenCorrected).withTimeout(2.0),

                // Drive to the outpost; shoot pre-loaded fuel concurrently.
                // deadline() ends when driveToOutpost finishes — shoot is cancelled then.
                Commands.deadline(driveToOutpost, shootWhileDriving),

                // Hold position while HUMAN PLAYER loads the chute.
                Commands.waitSeconds(OutpostConstants.OUTPOST_WAIT_SECONDS),

                // Credit the robot with a full chute of fuel.
                Commands.runOnce(() -> superstructure.setBallCount(
                        OutpostConstants.OUTPOST_CHUTE_CAPACITY)),

                // Shoot everything received from the outpost.
                shootReceived
            );
        }, Set.of(drivetrain, superstructure, vision))
        .finallyDo(interrupted -> {
            // Always return to STOWED on exit so the robot is safe.
            if (superstructure.getState() != RobotState.STOWED) {
                superstructure.requestState(RobotState.STOWED);
            }
        });
    }
}
