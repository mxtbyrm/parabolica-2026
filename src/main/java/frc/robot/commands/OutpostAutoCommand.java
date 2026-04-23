package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.OutpostConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Autonomous routine: deploy intake → pathfind to OUTPOST while shooting
 * on the move → keep shooting at the OUTPOST while the HUMAN PLAYER loads
 * the hopper directly.
 *
 * <h2>Sequence</h2>
 * <ol>
 *   <li><b>Deploy intake arm</b> — arm deploys at match start so balls on the
 *       floor can enter the robot during the drive.  Roller is intentionally
 *       NOT run — the human player fills the hopper directly at the OUTPOST.</li>
 *   <li><b>Drive + shoot on the move</b> — pathfinds to the OUTPOST dock pose
 *       while {@link ShootCommand} tracks and fires at the HUB with full
 *       moving-while-shooting compensation.  The pathfind is the deadline:
 *       as soon as the robot arrives, the drive phase ends.</li>
 *   <li><b>Shoot at OUTPOST</b> — credits a full chute of balls and fires
 *       continuously while the human player loads the hopper.  Runs for
 *       {@link OutpostConstants#OUTPOST_RECEIVE_SHOOT_TIMEOUT_S} seconds.</li>
 * </ol>
 *
 * <p>Phase is published to {@code SmartDashboard/"OutpostAuto/Phase"}.
 * Target and start poses are published for field-side diagnostics.
 */
public class OutpostAutoCommand {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        3.0,                             // max velocity m/s
        2.5,                             // max acceleration m/s²
        Units.degreesToRadians(540),     // max angular velocity rad/s
        Units.degreesToRadians(720)      // max angular acceleration rad/s²
    );

    private OutpostAutoCommand() {}

    /**
     * Creates the OUTPOST autonomous command.
     *
     * <p>Alliance color is resolved lazily at schedule time so the correct
     * dock pose is always used.
     *
     * @param drivetrain     Swerve drivetrain (pathfinding + SOTM velocity reads).
     * @param superstructure Flywheel, hood, turret, feeder, and spindexer coordinator.
     * @param vision         Limelight for HUB distance and angle.
     * @param photonVision   Corner cameras for global pose correction.
     * @param intake         Intake subsystem — arm deploys at start, stows on end.
     * @return The OUTPOST auto command.
     */
    public static Command create(
            CommandSwerveDrivetrain drivetrain,
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
                        ? FieldLayout.RED_OUTPOST_DOCK_POSE
                        : FieldLayout.BLUE_OUTPOST_DOCK_POSE;

                SmartDashboard.putString("OutpostAuto/Alliance",    isRed ? "Red" : "Blue");
                SmartDashboard.putString("OutpostAuto/TargetPose",
                        String.format("X=%.2f Y=%.2f", outpostPose.getX(), outpostPose.getY()));

                return Commands.sequence(

                    // 0 — Deploy intake immediately so the arm is down during the
                    //     vision wait and the entire drive phase.  Roller is not
                    //     run — the human player fills the hopper at the OUTPOST.
                    Commands.runOnce(() -> {
                        SmartDashboard.putString("OutpostAuto/Phase", "0-DeployIntake");
                        intake.deploy();
                    }),

                    // 1 — Confirm the drivetrain pose. prepareVisionForAuto() already
                    //     hard-reset it from the disabled-period vision fix, so no
                    //     blocking wait is needed here. We just log the active pose.
                    Commands.runOnce(() -> {
                        Pose2d resetPose = photonVision.getLatestRawPose()
                                .orElseGet(() -> drivetrain.getState().Pose);
                        drivetrain.resetPose(resetPose);
                        SmartDashboard.putString("OutpostAuto/Phase", "1-PoseInit");
                        SmartDashboard.putBoolean("OutpostAuto/VisionConfident",
                                photonVision.hasPoseBeenCorrected());
                        SmartDashboard.putString("OutpostAuto/InitPose",
                                String.format("X=%.2f Y=%.2f", resetPose.getX(), resetPose.getY()));
                    }),

                    // 2 — Drive to OUTPOST while shooting on the move.
                    //     pathfindToPose is the deadline — ends when robot arrives.
                    //     AutoShootCommand tracks the HUB and fires throughout the drive.
                    Commands.runOnce(() ->
                        SmartDashboard.putString("OutpostAuto/Phase", "2-DriveAndShoot")),

                    Commands.deadline(
                        AutoBuilder.pathfindToPose(outpostPose, CONSTRAINTS),
                        new ShootCommand(superstructure, vision, drivetrain, photonVision,
                                intake, () -> false)
                                .withTimeout(OutpostConstants.OUTPOST_DRIVE_SHOOT_TIMEOUT_S)
                    ),

                    // 3 — At OUTPOST: credit full chute and keep shooting while
                    //     the human player loads the hopper directly.
                    Commands.runOnce(() -> {
                        SmartDashboard.putString("OutpostAuto/Phase", "3-ShootAtOutpost");
                        superstructure.setBallCount(OutpostConstants.OUTPOST_CHUTE_CAPACITY);
                    }),
                    new ShootCommand(superstructure, vision, drivetrain, photonVision,
                            intake, () -> false)
                            .withTimeout(OutpostConstants.OUTPOST_RECEIVE_SHOOT_TIMEOUT_S),

                    Commands.runOnce(() ->
                        SmartDashboard.putString("OutpostAuto/Phase", "Done"))
                );
            },
            Set.of(drivetrain, superstructure, vision, intake)
        ).finallyDo(interrupted -> {
            superstructure.requestState(RobotState.STOWED);
            SmartDashboard.putString("OutpostAuto/Phase",
                    interrupted ? "Interrupted" : "Complete");
        });
    }
}
