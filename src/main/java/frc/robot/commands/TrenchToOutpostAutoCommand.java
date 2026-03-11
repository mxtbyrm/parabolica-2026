package frc.robot.commands;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.OutpostConstants;
import frc.robot.Constants.TrenchConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.TrenchTraversalManager;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Autonomous routine: continuous non-stop motion from the starting position
 * through the trench to the neutral zone, collect balls, return through the
 * trench, and arrive at the OUTPOST &mdash; the robot only stops at the very
 * start (pose init) and at the outpost.
 *
 * <h2>Architecture</h2>
 * <p>A single pre-built {@link PathPlannerPath} covers the entire route from the
 * neutral-zone trench exit to the outpost (6 waypoints).
 * {@link AutoBuilder#pathfindThenFollowPath} pathfinds through the outbound trench
 * to the path start, then seamlessly continues along the path without ever
 * decelerating to zero.</p>
 *
 * <p>{@link TrenchTraversalManager} auto-manages {@code TRAVERSING_TRENCH} state
 * in both directions based on robot position.  No explicit
 * {@code PassThroughTrenchCommand} is needed.</p>
 *
 * <h2>Parallel Branches (position-triggered)</h2>
 * <ul>
 *   <li><b>Intake:</b> deploys + runs rollers after the first trench exit.</li>
 *   <li><b>Shooting:</b> starts after the second (return) trench exit, so the
 *       robot shoots on the move from trench exit to outpost.</li>
 * </ul>
 *
 * <h2>Path Waypoints</h2>
 * <ol start="0">
 *   <li>Neutral-zone-side trench exit (outbound)</li>
 *   <li>Collection start (facing left wall)</li>
 *   <li>Collection end</li>
 *   <li>Return trench entry (opposite-wall heading)</li>
 *   <li>Return trench exit</li>
 *   <li>Outpost (v=0, outpost heading)</li>
 * </ol>
 *
 * <h2>Post-path</h2>
 * <p>At the outpost the human player feeds all 25 balls and the robot keeps
 * shooting.</p>
 *
 * <p>Uses the trench closest to each alliance's OUTPOST: bottom-wall (Blue
 * index 0) / top-wall (Red index 1).
 */
public class TrenchToOutpostAutoCommand {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        3.0,                             // max velocity m/s
        2.5,                             // max acceleration m/s²
        Units.degreesToRadians(540),     // max angular velocity rad/s
        Units.degreesToRadians(720)      // max angular acceleration rad/s²
    );

    /** Slower constraints for driving under the trench structure. */
    private static final PathConstraints TRENCH_CONSTRAINTS = new PathConstraints(
        2.0,                             // max velocity m/s
        2.0,                             // max acceleration m/s²
        Units.degreesToRadians(360),     // max angular velocity rad/s
        Units.degreesToRadians(540)      // max angular acceleration rad/s²
    );

    /** Robot-centric forward speed during neutral zone collection (m/s). */
    private static final double COLLECT_SPEED_MPS = 1.5;

    /** Duration of the forward collection drive (seconds). */
    private static final double COLLECT_DRIVE_S = 2.5;

    /** Constraints for the collection phase (slower, controlled pickup). */
    private static final PathConstraints COLLECT_CONSTRAINTS = new PathConstraints(
        COLLECT_SPEED_MPS,               // max velocity m/s
        1.5,                             // max acceleration m/s²
        Units.degreesToRadians(360),     // max angular velocity rad/s
        Units.degreesToRadians(540)      // max angular acceleration rad/s²
    );

    /**
     * Y-axis clearance past the trench outer edge for the collection start pose.
     */
    private static final double TRENCH_Y_CLEARANCE_M = 0.85;

    private TrenchToOutpostAutoCommand() {}

    /** Compute the tangent angle (direction of travel) from one pose to another. */
    private static Rotation2d tangentBetween(Pose2d from, Pose2d to) {
        return new Rotation2d(to.getX() - from.getX(), to.getY() - from.getY());
    }

    /**
     * Creates the trench-to-outpost autonomous command.
     *
     * @param drivetrain     Swerve drivetrain.
     * @param superstructure Scoring coordinator.
     * @param vision         Limelight vision.
     * @param photonVision   PhotonVision cameras.
     * @param intake         Intake subsystem.
     * @return The auto command.
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

                // RIGHT trench (from driver perspective): Blue = bottom (index 0), Red = top (index 1).
                int trenchIdx = isRed ? 1 : 0;

                // Left wall heading: intake faces the left wall during collection.
                //   Blue left wall = +Y → 90°
                //   Red  left wall = −Y → −90°
                Rotation2d leftWallHeading = isRed
                        ? Rotation2d.fromDegrees(-90)
                        : Rotation2d.fromDegrees(90);

                // Opposite alliance wall heading: intake faces AWAY from
                // the alliance wall (toward the opponent side) during the
                // return trip through the trench.
                //   Blue → intake faces Red wall = 0° (facing +X)
                //   Red  → intake faces Blue wall = 180° (facing −X)
                Rotation2d oppositeWallHeading = isRed
                        ? Rotation2d.fromDegrees(180)
                        : Rotation2d.fromDegrees(0);

                // --- Outbound: neutral-zone-side trench exit.
                // Keep the default straight-through heading (Blue 0°, Red 180°)
                // so the robot stays aligned with the trench axis for the
                // entire traversal.  The turn to leftWallHeading happens AFTER
                // exiting, during the Phase-3 pathfind to collectStartPose.
                Pose2d trenchNeutralExitPose = isRed
                        ? FieldLayout.RED_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx];

                // --- Return trip poses: same XY as through-poses but with
                // opposite-wall rotation so the intake faces safely away from
                // the direction of travel through the trench. ---

                // Neutral-zone-side trench entrance for the return trip.
                Pose2d returnTrenchEntryPose;
                {
                    returnTrenchEntryPose = new Pose2d(
                            trenchNeutralExitPose.getX(),
                            trenchNeutralExitPose.getY(),
                            oppositeWallHeading);
                }

                // Hub-side trench exit for the return trip.
                Pose2d returnTrenchExitPose;
                {
                    Pose2d base = isRed
                            ? FieldLayout.RED_TRENCH_THROUGH_POSES[trenchIdx]
                            : FieldLayout.BLUE_TRENCH_THROUGH_POSES[trenchIdx];
                    returnTrenchExitPose = new Pose2d(
                            base.getX(), base.getY(), oppositeWallHeading);
                }

                // Collection start: at field midpoint X, offset Y to clear the
                // trench, facing the left wall (reuses leftWallHeading above).
                double neutralX = FieldLayout.FIELD_LENGTH_M / 2.0;
                double trenchEdge = TrenchConstants.TRENCH_TOTAL_WIDTH_M;

                double collectStartY = isRed
                        ? FieldLayout.FIELD_WIDTH_M - trenchEdge - TRENCH_Y_CLEARANCE_M
                        : trenchEdge + TRENCH_Y_CLEARANCE_M;

                Pose2d collectStartPose = new Pose2d(neutralX, collectStartY, leftWallHeading);

                // Collection end: same X, pushed further toward the left wall.
                double collectDistance = COLLECT_SPEED_MPS * COLLECT_DRIVE_S;
                double collectEndY = isRed
                        ? collectStartY - collectDistance
                        : collectStartY + collectDistance;
                Pose2d collectEndPose = new Pose2d(neutralX, collectEndY, leftWallHeading);

                Pose2d outpostPose = isRed
                        ? FieldLayout.RED_OUTPOST_DOCK_POSE
                        : FieldLayout.BLUE_OUTPOST_DOCK_POSE;

                SmartDashboard.putString("TrenchOutpostAuto/Alliance", isRed ? "Red" : "Blue");

                // ============================================================
                // Single continuous path: neutral-zone trench exit → collect
                // → return through trench → outpost.  The robot never stops
                // between the outbound trench exit and the outpost.
                //
                // Waypoints (6):  0 trenchNeutralExit
                //                 1 collectStart
                //                 2 collectEnd
                //                 3 returnTrenchEntry
                //                 4 returnTrenchExit
                //                 5 outpost
                //
                // RotationTargets control holonomic heading changes:
                //   0.5  → leftWallHeading      (rotate during transit to collect)
                //   2.0  → leftWallHeading      (hold through collection end)
                //   2.8  → oppositeWallHeading  (rotate BEFORE entering trench at 3.0)
                //   4.2  → oppositeWallHeading  (hold past trench exit)
                //
                // ConstraintsZones slow the robot:
                //   [1.0,2.0] → COLLECT_CONSTRAINTS  (during collection)
                //   [3.0,4.0] → TRENCH_CONSTRAINTS   (under the trench)
                // ============================================================

                // Tangent angles (travel direction) for bezier waypoints.
                Rotation2d collectTangent = isRed
                        ? Rotation2d.fromDegrees(-90)   // toward -Y (Red left wall)
                        : Rotation2d.fromDegrees(90);   // toward +Y (Blue left wall)
                Rotation2d t01  = tangentBetween(trenchNeutralExitPose, collectStartPose);
                Rotation2d t23  = tangentBetween(collectEndPose, returnTrenchEntryPose);
                Rotation2d tTr  = tangentBetween(returnTrenchEntryPose, returnTrenchExitPose);
                Rotation2d t45  = tangentBetween(returnTrenchExitPose, outpostPose);

                PathPlannerPath fullPath = new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(trenchNeutralExitPose.getTranslation(), t01),
                                new Pose2d(collectStartPose.getTranslation(), collectTangent),
                                new Pose2d(collectEndPose.getTranslation(), t23),
                                new Pose2d(returnTrenchEntryPose.getTranslation(), tTr),
                                new Pose2d(returnTrenchExitPose.getTranslation(), tTr),
                                new Pose2d(outpostPose.getTranslation(), t45)),
                        List.of(  // holonomic rotation targets
                                new RotationTarget(0.5, leftWallHeading),
                                new RotationTarget(2.0, leftWallHeading),
                                new RotationTarget(2.8, oppositeWallHeading),
                                new RotationTarget(4.2, oppositeWallHeading)),
                        List.of(),  // pointTowardsZones
                        List.of(  // constraintZones
                                new ConstraintsZone(1.0, 2.0, COLLECT_CONSTRAINTS),
                                new ConstraintsZone(3.0, 4.0, TRENCH_CONSTRAINTS)),
                        List.of(),  // eventMarkers
                        CONSTRAINTS,
                        null,  // idealStartingState — let pathfindThenFollowPath handle it
                        new GoalEndState(0, outpostPose.getRotation()),
                        false);
                fullPath.preventFlipping = true;

                return Commands.sequence(

                    // 0 — Seed odometry from a fresh PhotonVision fix.
                    Commands.waitUntil(photonVision::hasPoseBeenCorrected)
                            .withTimeout(OutpostConstants.VISION_POSE_INIT_TIMEOUT_S),
                    Commands.runOnce(() -> {
                        Pose2d resetPose = photonVision.getLatestRawPose()
                                .orElseGet(() -> drivetrain.getState().Pose);
                        drivetrain.resetPose(resetPose);
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "0-PoseInit");
                    }),

                    // 1–7 — NON-STOP from current position to the outpost.
                    //
                    // pathfindThenFollowPath:
                    //   a) Pathfinds through the outbound trench to the
                    //      neutral-zone-side exit (waypoint 0 of fullPath).
                    //   b) Seamlessly continues the pre-built path without
                    //      ever decelerating to zero: collect → return trench
                    //      → outpost.
                    //
                    // TrenchTraversalManager auto-manages TRAVERSING_TRENCH
                    // state in both directions (no PassThroughTrenchCommand).
                    // Intake stays deployed during the return trench traversal
                    // (oppositeWallHeading keeps the arm safe).
                    //
                    // Parallel branches:
                    //   • Intake: deploy + run rollers after first trench exit.
                    //   • Shoot:  start shooting after second trench exit.
                    Commands.runOnce(() ->
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "1-Running")),

                    Commands.deadline(
                        // ---- DEADLINE: continuous motion ----
                        AutoBuilder.pathfindThenFollowPath(fullPath, TRENCH_CONSTRAINTS),

                        // ---- PARALLEL: deploy intake after first trench exit ----
                        Commands.sequence(
                            Commands.waitUntil(() -> TrenchTraversalManager.isInsideTrench(
                                    drivetrain.getState().Pose)),
                            Commands.waitUntil(() -> !TrenchTraversalManager.isInsideTrench(
                                    drivetrain.getState().Pose)),
                            Commands.runOnce(() -> {
                                intake.deploy();
                                SmartDashboard.putString(
                                        "TrenchOutpostAuto/Phase", "2-DeployIntake");
                            }),
                            Commands.run(intake::runRoller, intake)
                        ),

                        // ---- PARALLEL: shoot on the move after second trench exit ----
                        Commands.sequence(
                            // First trench (outbound)
                            Commands.waitUntil(() -> TrenchTraversalManager.isInsideTrench(
                                    drivetrain.getState().Pose)),
                            Commands.waitUntil(() -> !TrenchTraversalManager.isInsideTrench(
                                    drivetrain.getState().Pose)),
                            // Second trench (return)
                            Commands.waitUntil(() -> TrenchTraversalManager.isInsideTrench(
                                    drivetrain.getState().Pose)),
                            Commands.waitUntil(() -> !TrenchTraversalManager.isInsideTrench(
                                    drivetrain.getState().Pose)),
                            Commands.runOnce(() ->
                                SmartDashboard.putString(
                                        "TrenchOutpostAuto/Phase", "7-ShootToOutpost")),
                            new AutoShootCommand(superstructure, vision, drivetrain,
                                    photonVision,
                                    OutpostConstants.TRENCH_TO_OUTPOST_DRIVE_SHOOT_TIMEOUT_S)
                        )
                    ),

                    // 8 — At outpost: credit full chute and keep shooting.
                    Commands.runOnce(() -> {
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "8-ShootAtOutpost");
                        superstructure.setBallCount(OutpostConstants.OUTPOST_CHUTE_CAPACITY);
                    }),
                    new AutoShootCommand(superstructure, vision, drivetrain, photonVision,
                            OutpostConstants.OUTPOST_RECEIVE_SHOOT_TIMEOUT_S),

                    Commands.runOnce(() ->
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "Done"))
                );
            },
            Set.of(drivetrain, superstructure, vision, intake)
        ).finallyDo(interrupted -> {
            intake.stopRoller();
            intake.stow();
            superstructure.requestState(RobotState.STOWED);
            SmartDashboard.putString("TrenchOutpostAuto/Phase",
                    interrupted ? "Interrupted" : "Complete");
        });
    }
}
