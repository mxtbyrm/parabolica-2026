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
 * trench, and arrive at the OUTPOST.
 *
 * <h2>Architecture</h2>
 * <p>A single pre-built {@link PathPlannerPath} covers the entire route,
 * starting at the hub-side trench staging pose (WP 0) and ending at the
 * outpost (WP 8). {@link AutoBuilder#pathfindThenFollowPath} pathfinds to
 * the hub-side trench staging pose, then seamlessly follows the pre-built
 * path without ever decelerating to zero.</p>
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
 * <h2>Path Waypoints (10 total, 0-indexed)</h2>
 * <ol start="0">
 *   <li>Hub-side trench staging (outbound entry) — robot stages here before the outbound trench</li>
 *   <li>Neutral-zone-side trench exit (after outbound trench)</li>
 *   <li>Collection start (facing left wall)</li>
 *   <li>Collection end (turnaround)</li>
 *   <li>Face alliance wall, 5% toward hub, mid-Y — return sweep offset (NEW)</li>
 *   <li>Return to collect-start Y at offset X (was WP4)</li>
 *   <li>Neutral-zone trench approach staging — offset {@code NEUTRAL_STAGE_OFFSET_M}
 *       from the trench edge (was WP5)</li>
 *   <li>Return trench entry (at trench edge, was WP6)</li>
 *   <li>Hub-side trench exit (after return trench, was WP7)</li>
 *   <li>Outpost (v=0, outpost heading, was WP8)</li>
 * </ol>
 *
 * <p>A staging pose is inserted immediately before <em>every</em> trench entry
 * so the robot always approaches the trench from a known aligned position.</p>
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

    /**
     * Distance (m) by which the neutral-zone approach-staging waypoint is offset
     * further into the neutral zone from the trench neutral-side edge.  This
     * prevents consecutive identical-XY waypoints in the PathPlanner bezier spline
     * while still giving the robot a dedicated staging point before every trench
     * entry.
     */
    private static final double NEUTRAL_STAGE_OFFSET_M = 0.40;

    /** Collection depth: 75% of hub-side trench exit → field centre distance. */
    private static final double COLLECT_DEPTH_FRACTION = 0.75;

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

                // Reversed collect heading — 180° from leftWallHeading.
                // Robot faces this at WP5 (return sweep end) so it is already
                // oriented 90° toward allianceWallHeading before staging.
                //   Blue → −90° (facing −Y, toward bottom wall)
                //   Red  →  90° (facing +Y, toward top wall)
                Rotation2d reversedCollectHeading = leftWallHeading.rotateBy(Rotation2d.k180deg);

                // Alliance wall heading — robot faces its own alliance wall.
                // Used from WP5 through WP7 so the robot drives intake-first
                // through the return trench and straight to the outpost.
                //   Blue → 180° (facing −X, toward Blue wall)
                //   Red  →   0° (facing +X, toward Red wall)
                Rotation2d allianceWallHeading = isRed
                        ? Rotation2d.fromDegrees(0)
                        : Rotation2d.k180deg;

                // --- Outbound: neutral-zone-side trench exit.
                // The FieldLayout pose already has the straight-through heading
                // (Blue 0°, Red 180°); the turn to leftWallHeading happens AFTER
                // exiting, during transit to collectStartPose.
                Pose2d trenchNeutralExitPose = isRed
                        ? FieldLayout.RED_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx];

                // --- Return trip poses: same XY as through-poses but with
                // allianceWallHeading so the robot drives intake-first (facing
                // its own wall) through the return trench. ---

                // Neutral-zone-side trench entrance for the return trip.
                Pose2d returnTrenchEntryPose;
                {
                    returnTrenchEntryPose = new Pose2d(
                            trenchNeutralExitPose.getX(),
                            trenchNeutralExitPose.getY(),
                            allianceWallHeading);
                }

                // Hub-side trench exit for the return trip.
                Pose2d returnTrenchExitPose;
                {
                    Pose2d base = isRed
                            ? FieldLayout.RED_TRENCH_THROUGH_POSES[trenchIdx]
                            : FieldLayout.BLUE_TRENCH_THROUGH_POSES[trenchIdx];
                    returnTrenchExitPose = new Pose2d(
                            base.getX(), base.getY(), allianceWallHeading);
                }

                // Collection start: X at COLLECT_DEPTH_FRACTION of the
                // hub-side trench exit → field centre distance, offset Y to
                // clear the trench, facing the left wall.
                double hubSideX    = returnTrenchExitPose.getX();
                double fieldCenterX = FieldLayout.FIELD_LENGTH_M / 2.0;
                double neutralX    = hubSideX + COLLECT_DEPTH_FRACTION * (fieldCenterX - hubSideX);
                double trenchEdge  = TrenchConstants.TRENCH_TOTAL_WIDTH_M;

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

                // Return collect path: X shifted 5% toward hub so the return sweep
                // doesn't exactly overlap the outbound sweep (WP2–WP3 are at neutralX).
                // WP4 (new) is at the same Y as WP3 (collectEndY) but shifted X toward hub.
                // WP5 (was WP4) is back at collectStartY with the same offset X.
                double returnX    = hubSideX + 0.95 * (neutralX - hubSideX);
                Pose2d returnWP4Pose = new Pose2d(returnX, collectEndPose.getY(),   allianceWallHeading);
                Pose2d returnWP5Pose = new Pose2d(returnX, collectStartPose.getY(), reversedCollectHeading);

                Pose2d outpostPose = isRed
                        ? FieldLayout.RED_OUTPOST_DOCK_POSE
                        : FieldLayout.BLUE_OUTPOST_DOCK_POSE;

                // Neutral-zone approach staging: placed NEUTRAL_STAGE_OFFSET_M further
                // into the neutral zone from the trench neutral edge.  Keeps consecutive
                // waypoints at distinct positions for PathPlanner's bezier spline.
                // The robot stages here before every return-trench entry.
                Pose2d returnTrenchNeutralStagePose = new Pose2d(
                        isRed ? trenchNeutralExitPose.getX() - NEUTRAL_STAGE_OFFSET_M
                              : trenchNeutralExitPose.getX() + NEUTRAL_STAGE_OFFSET_M,
                        trenchNeutralExitPose.getY(),
                        allianceWallHeading);

                SmartDashboard.putString("TrenchOutpostAuto/Alliance", isRed ? "Red" : "Blue");

                // ============================================================
                // Single continuous path — 10 waypoints (WP 0-9):
                //
                //   0  returnTrenchExitPose         hub-side staging (BEFORE outbound trench)
                //   1  trenchNeutralExitPose         neutral-side exit (after outbound trench)
                //   2  collectStartPose              collection start
                //   3  collectEndPose                collection end (turnaround)
                //   4  returnWP4Pose                 face alliance wall, 5% toward hub, same Y as WP3 (NEW)
                //   5  returnWP5Pose                 return collect-start Y at offset X (was WP4)
                //   6  returnTrenchNeutralStagePose  neutral-side staging (was WP5)
                //   7  returnTrenchEntryPose         at trench neutral edge (was WP6)
                //   8  returnTrenchExitPose          hub-side exit (was WP7)
                //   9  outpostPose                   (was WP8)
                //
                // A staging pose is placed immediately before EVERY trench entry
                // (WP 0 for outbound, WP 6+7 for return) so the robot always
                // approaches the trench from a known aligned position.
                //
                // RotationTargets (path t = 0.0 to 9.0):
                //   1.5 → leftWallHeading          rotate mid-transit WP1→WP2
                //   3.0 → leftWallHeading          hold at WP3 (collection end)
                //   3.5 → allianceWallHeading      90° turn to own wall WP3→WP4
                //   4.5 → allianceWallHeading      hold through WP4→WP5
                //   6.2 → allianceWallHeading      hold through WP6→WP7
                //   8.2 → allianceWallHeading      hold through WP7→WP8
                //
                // ConstraintZones:
                //   [0.0, 1.0] TRENCH_CONSTRAINTS  outbound trench  (WP0→WP1)
                //   [2.0, 3.0] COLLECT_CONSTRAINTS outbound collect (WP2→WP3)
                //   [3.0, 5.0] COLLECT_CONSTRAINTS return collect   (WP3→WP4→WP5)
                //   [7.0, 8.0] TRENCH_CONSTRAINTS  return trench    (WP7→WP8)
                // ============================================================

                // Tangent angles (travel direction) for bezier waypoints.
                Rotation2d collectTangent    = isRed
                        ? Rotation2d.fromDegrees(-90)   // toward -Y (Red left wall)
                        : Rotation2d.fromDegrees(90);   // toward +Y (Blue left wall)
                Rotation2d collectTangentRev = collectTangent.rotateBy(Rotation2d.k180deg);

                Rotation2d tTrOut = tangentBetween(returnTrenchExitPose, trenchNeutralExitPose);
                Rotation2d t12    = tangentBetween(trenchNeutralExitPose, collectStartPose);
                Rotation2d t56r   = tangentBetween(returnWP5Pose, returnTrenchNeutralStagePose);   // WP5→WP6
                Rotation2d t56    = tangentBetween(returnTrenchNeutralStagePose, returnTrenchEntryPose); // WP6→WP7
                Rotation2d tTrRet = tangentBetween(returnTrenchEntryPose, returnTrenchExitPose);  // WP7→WP8
                Rotation2d t78    = tangentBetween(returnTrenchExitPose,  outpostPose);            // WP8→WP9

                PathPlannerPath fullPath = new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(
                                // WP0 — hub-side staging before outbound trench
                                new Pose2d(returnTrenchExitPose.getTranslation(),         tTrOut),
                                // WP1 — neutral-side exit after outbound trench
                                new Pose2d(trenchNeutralExitPose.getTranslation(),        t12),
                                // WP2 — collection start
                                new Pose2d(collectStartPose.getTranslation(),             collectTangent),
                                // WP3 — collection end (turnaround)
                                new Pose2d(collectEndPose.getTranslation(),               collectTangentRev),
                                // WP4 — turn to alliance wall, 5% toward hub, mid-Y (NEW)
                                new Pose2d(returnWP4Pose.getTranslation(),                collectTangentRev),
                                // WP5 — return to collect-start Y at offset X (was WP4)
                                new Pose2d(returnWP5Pose.getTranslation(),                t56r),
                                // WP6 — neutral-zone staging before return trench (was WP5)
                                new Pose2d(returnTrenchNeutralStagePose.getTranslation(), t56),
                                // WP7 — at trench neutral edge, aligned for return (was WP6)
                                new Pose2d(returnTrenchEntryPose.getTranslation(),        tTrRet),
                                // WP8 — hub-side exit after return trench (was WP7)
                                new Pose2d(returnTrenchExitPose.getTranslation(),         t78),
                                // WP9 — outpost (was WP8)
                                new Pose2d(outpostPose.getTranslation(),                  t78)),
                        List.of(  // holonomic rotation targets
                                new RotationTarget(1.5, leftWallHeading),
                                new RotationTarget(3.0, leftWallHeading),
                                new RotationTarget(3.5, allianceWallHeading),      // 90° turn to own wall at WP4
                                new RotationTarget(4.5, reversedCollectHeading), // face opposite side wall at WP5
                                new RotationTarget(6.2, allianceWallHeading),    // face own wall by WP6 staging
                                new RotationTarget(8.2, allianceWallHeading)),   // hold through WP7→WP8
                        List.of(),  // pointTowardsZones
                        List.of(  // constraintZones
                                new ConstraintsZone(0.0, 1.0, TRENCH_CONSTRAINTS),
                                new ConstraintsZone(2.0, 3.0, COLLECT_CONSTRAINTS),   // outbound collect
                                new ConstraintsZone(3.0, 5.0, COLLECT_CONSTRAINTS),   // return collect
                                new ConstraintsZone(7.0, 8.0, TRENCH_CONSTRAINTS)),   // return trench
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

                    // 1–8 — NON-STOP from current position to the outpost.
                    //
                    // pathfindThenFollowPath:
                    //   a) Pathfinds to WP0 (hub-side trench staging pose).
                    //   b) Seamlessly continues the pre-built path:
                    //      outbound trench → collect → turnaround → neutral
                    //      staging → return trench → outpost.
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
                        // Pathfind to WP0 (hub-side staging) using full speed;
                        // the pre-built path's constraint zones slow the robot
                        // for both trench traversals and the collection sweep.
                        AutoBuilder.pathfindThenFollowPath(fullPath, CONSTRAINTS),

                        // ---- PARALLEL: intake WP1 → WP6 ----
                        // Deploy + run rollers after outbound trench exit (WP1).
                        // Stop rollers and stow when the robot enters the return
                        // trench at WP6 (second isInsideTrench event).
                        Commands.sequence(
                            // Wait for outbound trench exit (WP1)
                            Commands.waitUntil(() -> TrenchTraversalManager.isInsideTrench(
                                    drivetrain.getState().Pose)),
                            Commands.waitUntil(() -> !TrenchTraversalManager.isInsideTrench(
                                    drivetrain.getState().Pose)),
                            Commands.runOnce(() -> {
                                intake.deploy();
                                SmartDashboard.putString(
                                        "TrenchOutpostAuto/Phase", "2-IntakeRunning");
                            }),
                            // Run rollers until return trench entry (WP7)
                            Commands.deadline(
                                Commands.waitUntil(() -> TrenchTraversalManager.isInsideTrench(
                                        drivetrain.getState().Pose)),
                                Commands.run(intake::runRoller, intake)
                            ),
                            Commands.runOnce(() -> {
                                intake.stopRoller();
                                SmartDashboard.putString(
                                        "TrenchOutpostAuto/Phase", "6-RollerStopped");
                            })
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
                    //     Commands.run(() -> {}, drivetrain) holds the drivetrain
                    //     requirement so no default command can move the robot;
                    //     CTRE persists the last applied control (v=0 from
                    //     GoalEndState) so the wheels stay still throughout.
                    Commands.runOnce(() -> {
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "8-ShootAtOutpost");
                        superstructure.setBallCount(OutpostConstants.OUTPOST_CHUTE_CAPACITY);
                    }),
                    Commands.deadline(
                        new AutoShootCommand(superstructure, vision, drivetrain, photonVision,
                                OutpostConstants.OUTPOST_RECEIVE_SHOOT_TIMEOUT_S),
                        Commands.run(() -> {}, drivetrain)
                    ),

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
