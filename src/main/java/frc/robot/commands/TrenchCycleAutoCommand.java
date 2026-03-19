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
 * Autonomous routine: continuous trench cycling — collect in the neutral zone,
 * return through the trench, shoot for {@value #SHOOT_TIMEOUT_S} seconds, then
 * repeat until auto ends.  No outpost visit.
 *
 * <h2>Variants</h2>
 * <ul>
 *   <li>{@link Side#RIGHT} — trench closest to the OUTPOST side:
 *       Blue index 0 (bottom wall), Red index 1 (top wall).</li>
 *   <li>{@link Side#LEFT}  — trench closest to the alliance wall:
 *       Blue index 1 (top wall),  Red index 0 (bottom wall).</li>
 * </ul>
 *
 * <h2>Cycle path — 8 waypoints (WP 0–7, no outpost)</h2>
 * <ol start="0">
 *   <li>Hub-side trench staging (outbound entry)</li>
 *   <li>Neutral-zone-side trench exit</li>
 *   <li>Collection start (facing left wall)</li>
 *   <li>Collection end</li>
 *   <li>Collection start again (turnaround)</li>
 *   <li>Neutral-zone trench approach staging ({@value #NEUTRAL_STAGE_OFFSET_M} m offset)</li>
 *   <li>Return trench entry</li>
 *   <li>Hub-side trench exit — {@link GoalEndState} v=0, facing opposite wall
 *       so the robot is immediately ready for the next outbound trip</li>
 * </ol>
 *
 * <h2>Per-cycle sequence</h2>
 * <ol>
 *   <li>pathfindThenFollowPath to WP0, then follow WP0→WP7.</li>
 *   <li>Intake runs WP1 (outbound exit) → WP6 (return entry); roller stops, intake stays deployed.</li>
 *   <li>After path: {@link AutoShootCommand} for {@value #SHOOT_TIMEOUT_S} s.</li>
 *   <li>Repeat from step 1 until auto ends.</li>
 * </ol>
 */
public class TrenchCycleAutoCommand {

    /** Which trench to cycle through. */
    public enum Side { LEFT, RIGHT }

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        3.0,
        2.5,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    private static final PathConstraints TRENCH_CONSTRAINTS = new PathConstraints(
        2.0,
        2.0,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540)
    );

    private static final double COLLECT_SPEED_MPS = 1.5;
    private static final double COLLECT_DRIVE_S   = 2.5;

    private static final PathConstraints COLLECT_CONSTRAINTS = new PathConstraints(
        COLLECT_SPEED_MPS,
        1.5,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540)
    );

    private static final double TRENCH_Y_CLEARANCE_M   = 0.85;
    private static final double NEUTRAL_STAGE_OFFSET_M = 0.40;

    /** How long to shoot after each return through the trench. */
    private static final double SHOOT_TIMEOUT_S = 6.0;

    /** Collection depth fraction for cycle 1 (75% of hub→centre distance — deeper sweep). */
    private static final double CYCLE_1_DEPTH_FRACTION = 0.75;

    /** Collection depth fraction for cycle 2 (50% of hub→centre distance — middle sweep). */
    private static final double CYCLE_2_DEPTH_FRACTION = 0.50;

    private TrenchCycleAutoCommand() {}

    private static Rotation2d tangentBetween(Pose2d from, Pose2d to) {
        return new Rotation2d(to.getX() - from.getX(), to.getY() - from.getY());
    }

    /**
     * Creates the repeating trench-cycle autonomous command.
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
                boolean isRed  = DriverStation.getAlliance()
                        .map(a -> a == DriverStation.Alliance.Red)
                        .orElse(false);
                boolean isRight = (side == Side.RIGHT);

                // Trench index — which FieldLayout array entry to use.
                // RIGHT: Blue=0 (bottom wall), Red=1 (top wall)
                // LEFT:  Blue=1 (top wall),   Red=0 (bottom wall)
                int trenchIdx = isRight ? (isRed ? 1 : 0) : (isRed ? 0 : 1);

                // Collection sweep direction.
                // RIGHT+Blue or LEFT+Red → sweep +Y; otherwise −Y.
                boolean sweepPositiveY = isRight != isRed;

                // Robot heading during collection: faces the wall being swept toward.
                Rotation2d leftWallHeading = Rotation2d.fromDegrees(sweepPositiveY ? 90 : -90);

                // 180° from leftWallHeading — robot turns after collection end (WP3)
                // and holds this through WP4 while rollers keep running.
                Rotation2d reversedCollectHeading = leftWallHeading.rotateBy(Rotation2d.k180deg);

                // Return heading: faces the OPPONENT alliance wall so the robot is
                // already oriented for the next outbound trip without an extra spin.
                //   Blue → 0°  (+X, toward Red wall)
                //   Red  → 180° (−X, toward Blue wall)
                Rotation2d returnHeading = isRed
                        ? Rotation2d.k180deg
                        : Rotation2d.fromDegrees(0);

                // ── Field poses ───────────────────────────────────────────────
                Pose2d trenchNeutralExitPose = isRed
                        ? FieldLayout.RED_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx];

                Pose2d hubSideBase = isRed
                        ? FieldLayout.RED_TRENCH_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_THROUGH_POSES[trenchIdx];

                // WP0 = WP7: hub-side staging / cycle end, both using returnHeading.
                Pose2d returnTrenchExitPose = new Pose2d(
                        hubSideBase.getX(), hubSideBase.getY(), returnHeading);

                // Return trench entry (same XY as outbound exit, returnHeading).
                Pose2d returnTrenchEntryPose = new Pose2d(
                        trenchNeutralExitPose.getX(),
                        trenchNeutralExitPose.getY(),
                        returnHeading);

                // Neutral-zone staging before return trench entry.
                Pose2d returnTrenchNeutralStagePose = new Pose2d(
                        isRed ? trenchNeutralExitPose.getX() - NEUTRAL_STAGE_OFFSET_M
                              : trenchNeutralExitPose.getX() + NEUTRAL_STAGE_OFFSET_M,
                        trenchNeutralExitPose.getY(),
                        returnHeading);

                // Collection Y coordinates (shared across cycles — only X varies).
                double trenchEdge     = TrenchConstants.TRENCH_TOTAL_WIDTH_M;
                double collectStartY  = sweepPositiveY
                        ? trenchEdge + TRENCH_Y_CLEARANCE_M
                        : FieldLayout.FIELD_WIDTH_M - trenchEdge - TRENCH_Y_CLEARANCE_M;
                double collectDistance = COLLECT_SPEED_MPS * COLLECT_DRIVE_S;
                double collectEndY    = sweepPositiveY
                        ? collectStartY + collectDistance
                        : collectStartY - collectDistance;

                // Hub-side X reference and field centre X — used to compute
                // per-cycle collection depth as a fraction of this range.
                double hubSideX    = returnTrenchExitPose.getX();
                double fieldCenterX = FieldLayout.FIELD_LENGTH_M / 2.0;

                SmartDashboard.putString("TrenchCycleAuto/Alliance", isRed ? "Red" : "Blue");
                SmartDashboard.putString("TrenchCycleAuto/Side",     side.name());

                // ── Shared path tangents (not dependent on collection X) ──────
                Rotation2d collectTangent    = Rotation2d.fromDegrees(sweepPositiveY ? 90 : -90);
                Rotation2d collectTangentRev = collectTangent.rotateBy(Rotation2d.k180deg);

                Rotation2d tTrOut = tangentBetween(returnTrenchExitPose,          trenchNeutralExitPose);
                Rotation2d t56    = tangentBetween(returnTrenchNeutralStagePose,   returnTrenchEntryPose);
                Rotation2d tTrRet = tangentBetween(returnTrenchEntryPose,          returnTrenchExitPose);

                // ── Per-cycle builder ─────────────────────────────────────────
                // Builds a fresh cycle path + command for the given depth fraction.
                // depthFraction: fraction of the hubSideX→fieldCenterX distance.
                //   0.75 → deeper sweep (cycle 1)
                //   0.50 → middle sweep (cycle 2)
                //
                // RotationTargets (t = 0.0 … 7.0):
                //   1.5 → leftWallHeading          rotate mid-transit WP1→WP2
                //   3.0 → leftWallHeading          hold at WP3 (collection end)
                //   3.7 → reversedCollectHeading   180° spin just after WP3
                //   4.5 → reversedCollectHeading   hold at WP4 (rollers still running)
                //   5.2 → returnHeading            face opponent wall by WP5
                //   6.5 → returnHeading            hold through WP6, WP7
                //
                // ConstraintZones:
                //   [0.0, 1.0] TRENCH_CONSTRAINTS  outbound trench (WP0→WP1)
                //   [2.0, 3.0] COLLECT_CONSTRAINTS collection sweep (WP2→WP3)
                //   [6.0, 7.0] TRENCH_CONSTRAINTS  return trench   (WP6→WP7)
                java.util.function.Function<Double, Command> buildCycle = (depthFraction) -> {
                    double neutralX = hubSideX + depthFraction * (fieldCenterX - hubSideX);

                    Pose2d collectStartPose = new Pose2d(neutralX, collectStartY, leftWallHeading);
                    Pose2d collectEndPose   = new Pose2d(neutralX, collectEndY,   leftWallHeading);

                    // Tangents that depend on collection X position
                    Rotation2d t12 = tangentBetween(trenchNeutralExitPose, collectStartPose);
                    Rotation2d t45 = tangentBetween(collectStartPose,      returnTrenchNeutralStagePose);

                    PathPlannerPath cyclePath = new PathPlannerPath(
                            PathPlannerPath.waypointsFromPoses(
                                    // WP0 — hub-side staging (before outbound trench)
                                    new Pose2d(returnTrenchExitPose.getTranslation(),        tTrOut),
                                    // WP1 — neutral-side exit (after outbound trench)
                                    new Pose2d(trenchNeutralExitPose.getTranslation(),        t12),
                                    // WP2 — collection start
                                    new Pose2d(collectStartPose.getTranslation(),             collectTangent),
                                    // WP3 — collection end (turnaround)
                                    new Pose2d(collectEndPose.getTranslation(),               collectTangentRev),
                                    // WP4 — back to collection start
                                    new Pose2d(collectStartPose.getTranslation(),             t45),
                                    // WP5 — neutral staging before return trench
                                    new Pose2d(returnTrenchNeutralStagePose.getTranslation(), t56),
                                    // WP6 — return trench entry
                                    new Pose2d(returnTrenchEntryPose.getTranslation(),        tTrRet),
                                    // WP7 — hub-side exit; v=0, returnHeading (ready for next cycle)
                                    new Pose2d(returnTrenchExitPose.getTranslation(),         tTrRet)),
                            List.of(
                                    new RotationTarget(1.5, leftWallHeading),
                                    new RotationTarget(3.0, leftWallHeading),
                                    new RotationTarget(3.7, reversedCollectHeading),
                                    new RotationTarget(4.5, reversedCollectHeading),
                                    new RotationTarget(5.2, returnHeading),
                                    new RotationTarget(6.5, returnHeading)),
                            List.of(),  // pointTowardsZones
                            List.of(
                                    new ConstraintsZone(0.0, 1.0, TRENCH_CONSTRAINTS),
                                    new ConstraintsZone(2.0, 3.0, COLLECT_CONSTRAINTS),
                                    new ConstraintsZone(6.0, 7.0, TRENCH_CONSTRAINTS)),
                            List.of(),  // eventMarkers
                            CONSTRAINTS,
                            null,  // idealStartingState — pathfindThenFollowPath handles it
                            new GoalEndState(0, returnHeading),
                            false);
                    cyclePath.preventFlipping = true;

                    return Commands.sequence(

                        Commands.runOnce(() -> {
                            SmartDashboard.putString("TrenchCycleAuto/Phase", "Cycling");
                            SmartDashboard.putNumber("TrenchCycleAuto/NeutralX", neutralX);
                            SmartDashboard.putNumber("TrenchCycleAuto/DepthFraction", depthFraction);
                        }),

                        Commands.deadline(
                            // ---- DEADLINE: follow the cycle path ----
                            AutoBuilder.pathfindThenFollowPath(cyclePath, CONSTRAINTS),

                            // ---- PARALLEL: intake WP1 → WP6 ----
                            Commands.sequence(
                                // Wait for outbound trench entry …
                                Commands.waitUntil(() -> TrenchTraversalManager.isInsideTrench(
                                        drivetrain.getState().Pose)),
                                // … then wait for outbound trench exit (WP1)
                                Commands.waitUntil(() -> !TrenchTraversalManager.isInsideTrench(
                                        drivetrain.getState().Pose)),
                                Commands.runOnce(() -> {
                                    intake.deploy();
                                    SmartDashboard.putString("TrenchCycleAuto/Phase", "Intaking");
                                }),
                                // Run rollers until return trench entry (WP6)
                                Commands.deadline(
                                    Commands.waitUntil(() -> TrenchTraversalManager.isInsideTrench(
                                            drivetrain.getState().Pose)),
                                    Commands.run(intake::runRoller, intake)
                                ),
                                Commands.runOnce(() -> {
                                    intake.stopRoller();
                                    SmartDashboard.putString("TrenchCycleAuto/Phase", "RollerStopped");
                                })
                            )
                        ),

                        // ---- Shoot after WP7 ----
                        Commands.runOnce(() ->
                            SmartDashboard.putString("TrenchCycleAuto/Phase", "Shooting")),
                        new AutoShootCommand(superstructure, vision, drivetrain, photonVision,
                                SHOOT_TIMEOUT_S)
                    );
                };

                // Each cycle uses Commands.defer so pathfindThenFollowPath and
                // AutoShootCommand are fresh instances per cycle.
                return Commands.sequence(

                    // ── 0: Vision pose init (once, before first cycle) ───────
                    // Wait for a fresh PhotonVision fix captured AFTER auto starts,
                    // then hard-reset odometry so pathfindThenFollowPath starts from
                    // the real physical location.  Falls back to the fused estimate
                    // after the timeout if no AprilTags are visible.
                    Commands.waitUntil(photonVision::hasPoseBeenCorrected)
                            .withTimeout(OutpostConstants.VISION_POSE_INIT_TIMEOUT_S),
                    Commands.runOnce(() -> {
                        Pose2d resetPose = photonVision.getLatestRawPose()
                                .orElseGet(() -> drivetrain.getState().Pose);
                        drivetrain.resetPose(resetPose);
                        SmartDashboard.putString("TrenchCycleAuto/Phase", "0-PoseInit");
                    }),

                    // ── Cycle 1 (75% depth — deeper sweep) ───────────────────
                    Commands.defer(() -> buildCycle.apply(CYCLE_1_DEPTH_FRACTION),
                            Set.of(drivetrain, superstructure, vision, intake)),

                    // ── Cycle 2 (50% depth — middle sweep) ───────────────────
                    // After the 6-second shoot at the end of cycle 2 the sequence
                    // finishes; CTRE persists the last applied control (v=0 from
                    // GoalEndState) so the robot stays at the hub-side trench exit.
                    Commands.defer(() -> buildCycle.apply(CYCLE_2_DEPTH_FRACTION),
                            Set.of(drivetrain, superstructure, vision, intake))
                );
            },
            Set.of(drivetrain, superstructure, vision, intake)
        ).finallyDo(interrupted -> {
            intake.stopRoller();
            intake.stow();
            superstructure.requestState(RobotState.STOWED);
            SmartDashboard.putString("TrenchCycleAuto/Phase",
                    interrupted ? "Interrupted" : "Complete");
        });
    }
}
