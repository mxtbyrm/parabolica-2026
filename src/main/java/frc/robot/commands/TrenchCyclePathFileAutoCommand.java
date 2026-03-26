package frc.robot.commands;

import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Autonomous routine: continuous trench cycling using PathPlanner GUI-drawn paths.
 *
 * <p>Identical logic to {@link TrenchCycleAutoCommand} but loads paths from
 * {@code .path} files instead of building them programmatically.
 *
 * <h2>Required path files</h2>
 * <p>Create four files in {@code src/main/deploy/pathplanner/paths/}:
 * <ul>
 *   <li>{@code TrenchCycleLeft1.path}  — outbound through left trench + collect + return to shoot pose</li>
 *   <li>{@code TrenchCycleLeft2.path}  — second cycle, shorter collect excursion</li>
 *   <li>{@code TrenchCycleRight1.path} — same for right trench</li>
 *   <li>{@code TrenchCycleRight2.path}</li>
 * </ul>
 *
 * <p>If you draw paths for one alliance only and rely on PathPlanner's alliance
 * flip, leave {@link PathPlannerPath#preventFlipping} at its default
 * ({@code false}).  If you draw separate files per alliance, name them
 * {@code TrenchCycleLeft1Blue.path} / {@code TrenchCycleLeft1Red.path} etc.
 * and set {@code preventFlipping = true} in the path options panel.
 *
 * <h2>Event markers to add in PathPlanner GUI</h2>
 * <p>Set these on each path using the same trigger positions as the programmatic version:
 * <ul>
 *   <li>{@code "StartRoller"} — near WP1 (trench exit) — registered in
 *       {@link frc.robot.RobotContainer#registerNamedCommands()}</li>
 *   <li>{@code "StopRoller"}  — before WP5 (return trench entry) — registered in
 *       {@link frc.robot.RobotContainer#registerNamedCommands()}</li>
 *   <li>{@code "StartShoot"}  — near WP5 (approaching shoot pose) — registered
 *       <em>per-cycle</em> inside this command's {@code Commands.defer()} lambda;
 *       <b>use the same name {@code "StartShoot"} on both paths</b></li>
 * </ul>
 *
 * <h2>Shoot-while-driving</h2>
 * <p>Same pattern as {@link TrenchCycleAutoCommand}: the {@code "StartShoot"} event
 * marker sets an {@link AtomicBoolean} flag that unblocks a {@code waitUntil},
 * after which {@link ShootCommand} runs in a {@link Commands#deadline} alongside
 * {@code pathfindThenFollowPath}.  ShootCommand requires only {@code superstructure}
 * and {@code vision} — no conflict with {@code drivetrain}.
 */
public class TrenchCyclePathFileAutoCommand {

    /** Which trench to cycle through. */
    public enum Side { LEFT, RIGHT }

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        4.0, 3.5,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    private TrenchCyclePathFileAutoCommand() {}

    /**
     * Creates the two-cycle trench autonomous command using pre-drawn PathPlanner paths.
     *
     * @param side           Which trench to use.
     * @param drivetrain     Swerve drivetrain.
     * @param superstructure Scoring coordinator.
     * @param vision         Limelight vision.
     * @param photonVision   PhotonVision cameras.
     * @param intake         Intake subsystem.
     * @return The auto command (runs until interrupted).
     */
    public static Command create(
            Side side,
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

                String sideName = (side == Side.LEFT) ? "Left" : "Right";

                // ── Load paths from PathPlanner GUI files ─────────────────────
                // Example names: "TrenchCycleLeft1", "TrenchCycleLeft2".
                // If you use alliance-specific files append "Blue"/"Red" here.
                PathPlannerPath cycle1Path;
                PathPlannerPath cycle2Path;
                try {
                    cycle1Path = PathPlannerPath.fromPathFile("TrenchCycle" + sideName + "1");
                    cycle2Path = PathPlannerPath.fromPathFile("TrenchCycle" + sideName + "2");
                } catch (Exception e) {
                    throw new RuntimeException("Failed to load TrenchCycle" + sideName + " paths", e);
                }

                // ── Per-cycle shoot flags ──────────────────────────────────────
                // "StartShoot" is re-registered every time the auto is scheduled
                // so each cycle gets a fresh, independent AtomicBoolean.
                // Cycle 1 runs first: we register shootFlag1 now.
                // Cycle 2 re-registers before its path starts (see below).
                AtomicBoolean shootFlag1 = new AtomicBoolean(false);
                AtomicBoolean shootFlag2 = new AtomicBoolean(false);

                SmartDashboard.putString("TrenchCycleAuto/Alliance", isRed ? "Red" : "Blue");
                SmartDashboard.putString("TrenchCycleAuto/Side",     side.name());

                return Commands.sequence(

                    // ── Vision pose init ──────────────────────────────────────
                    Commands.runOnce(() -> {
                        Pose2d resetPose = photonVision.getLatestRawPose()
                                .orElseGet(() -> drivetrain.getState().Pose);
                        drivetrain.resetPose(resetPose);
                        intake.deploy();
                        SmartDashboard.putString("TrenchCycleAuto/Phase", "0-PoseInit");
                    }),

                    // ── Cycle 1 ───────────────────────────────────────────────
                    Commands.runOnce(() -> {
                        shootFlag1.set(false);
                        // Bind "StartShoot" to cycle 1's flag BEFORE the path runs.
                        NamedCommands.registerCommand("StartShoot",
                                Commands.runOnce(() -> shootFlag1.set(true)));
                        SmartDashboard.putString("TrenchCycleAuto/Phase", "C1-Path");
                    }),
                    Commands.deadline(
                        Commands.sequence(
                            Commands.waitUntil(shootFlag1::get),
                            Commands.runOnce(() ->
                                SmartDashboard.putString("TrenchCycleAuto/Phase", "C1-Shoot")),
                            new ShootCommand(superstructure, vision, drivetrain, photonVision,
                                    intake, () -> false).withTimeout(6.0)
                        ),
                        AutoBuilder.pathfindThenFollowPath(cycle1Path, CONSTRAINTS)
                    ),

                    // ── Cycle 2 ───────────────────────────────────────────────
                    Commands.runOnce(() -> {
                        shootFlag2.set(false);
                        // Re-bind "StartShoot" to cycle 2's flag BEFORE the path runs.
                        NamedCommands.registerCommand("StartShoot",
                                Commands.runOnce(() -> shootFlag2.set(true)));
                        SmartDashboard.putString("TrenchCycleAuto/Phase", "C2-Path");
                    }),
                    Commands.deadline(
                        Commands.sequence(
                            Commands.waitUntil(shootFlag2::get),
                            Commands.runOnce(() ->
                                SmartDashboard.putString("TrenchCycleAuto/Phase", "C2-Shoot")),
                            new ShootCommand(superstructure, vision, drivetrain, photonVision,
                                    intake, () -> false).withTimeout(6.0)
                        ),
                        AutoBuilder.pathfindThenFollowPath(cycle2Path, CONSTRAINTS)
                    )
                );
            },
            Set.of(drivetrain, superstructure, vision, intake)
        ).finallyDo(interrupted -> {
            intake.stopRoller();
            superstructure.requestState(RobotState.STOWED);
            SmartDashboard.putString("TrenchCycleAuto/Phase",
                    interrupted ? "Interrupted" : "Complete");
        });
    }
}
