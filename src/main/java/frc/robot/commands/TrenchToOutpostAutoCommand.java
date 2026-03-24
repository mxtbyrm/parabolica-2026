package frc.robot.commands;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Autonomous routine: through trench → collect → return → shoot on the move → OUTPOST.
 *
 * <h2>Path Waypoints (8 total, WP 0–7)</h2>
 * <ol start="0">
 *   <li>Trench centre (outbound entry)</li>
 *   <li>Neutral-zone trench exit (outbound)</li>
 *   <li>Collection start</li>
 *   <li>Collection end ({@value #COLLECT_SWEEP_Y_M} m sweep)</li>
 *   <li>Collection start (return; same XY as WP2)</li>
 *   <li>Neutral-zone trench exit (return)</li>
 *   <li>Hub-side trench exit</li>
 *   <li>Outpost ({@link GoalEndState} v=0)</li>
 * </ol>
 *
 * <h2>Heading conventions</h2>
 * <ul>
 *   <li>All trench transits (WP0–WP1, WP5–WP6): face opponent alliance wall.</li>
 *   <li>Collection (WP2–WP4): face the collection wall.</li>
 * </ul>
 */
public class TrenchToOutpostAutoCommand {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        3.0, 2.5,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    private static final PathConstraints TRENCH_CONSTRAINTS = new PathConstraints(
        2.0, 2.0,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540)
    );

    private static final PathConstraints COLLECT_CONSTRAINTS = new PathConstraints(
        1.5, 1.5,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540)
    );

    private static final double TRENCH_Y_CLEARANCE_M   = 0.85;
    private static final double ROBOT_HALF_WIDTH_M      = Units.inchesToMeters(14.5);
    private static final double COLLECT_SAFETY_MARGIN_M = 0.15;
    /** Y distance swept during collection (hand-written; same value as TrenchCycleAutoCommand). */
    private static final double COLLECT_SWEEP_Y_M      = 2.0;

    private TrenchToOutpostAutoCommand() {}

    private static Rotation2d tangentBetween(Translation2d from, Pose2d to) {
        return new Rotation2d(to.getX() - from.getX(), to.getY() - from.getY());
    }

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

                // Outpost-side trench: Blue = bottom-wall (index 0), Red = top-wall (index 1).
                int trenchIdx = isRed ? 1 : 0;

                // Outbound trench transit: face opponent alliance wall.
                //   Blue → 0°   (facing +X, toward Red wall)
                //   Red  → 180° (facing -X, toward Blue wall)
                Rotation2d returnHeading = isRed
                        ? Rotation2d.k180deg
                        : Rotation2d.fromDegrees(0);

                // Return trench transit + outpost approach: face own alliance wall.
                //   Blue → 180° (facing -X, toward Blue wall)
                //   Red  →   0° (facing +X, toward Red wall)
                Rotation2d allianceWallHeading = isRed
                        ? Rotation2d.fromDegrees(0)
                        : Rotation2d.k180deg;

                // Collection heading: face the wall being swept toward.
                //   Blue bottom trench → sweep +Y → face 90°
                //   Red  top    trench → sweep -Y → face -90°
                Rotation2d leftWallHeading = isRed
                        ? Rotation2d.fromDegrees(-90)
                        : Rotation2d.fromDegrees(90);
                Rotation2d collectTangent    = leftWallHeading;
                Rotation2d collectTangentRev = collectTangent.rotateBy(Rotation2d.k180deg);

                // ── Field poses ───────────────────────────────────────────────
                Translation2d trenchCenter = isRed
                        ? FieldLayout.TRENCH_RED_CENTERS[trenchIdx]
                        : FieldLayout.TRENCH_BLUE_CENTERS[trenchIdx];

                Pose2d trenchNeutralExitPose = isRed
                        ? FieldLayout.RED_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx];

                Pose2d hubSideExitPose = isRed
                        ? FieldLayout.RED_TRENCH_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_THROUGH_POSES[trenchIdx];

                Pose2d outpostPose = isRed
                        ? FieldLayout.RED_OUTPOST_DOCK_POSE
                        : FieldLayout.BLUE_OUTPOST_DOCK_POSE;

                // ── Collection geometry ───────────────────────────────────────
                // Collect X: robot centre just shy of the field centre line.
                double fieldCenterX = FieldLayout.FIELD_LENGTH_M / 2.0;
                double neutralX     = isRed
                        ? fieldCenterX + ROBOT_HALF_WIDTH_M + COLLECT_SAFETY_MARGIN_M
                        : fieldCenterX - ROBOT_HALF_WIDTH_M - COLLECT_SAFETY_MARGIN_M;
                double trenchEdge   = TrenchConstants.TRENCH_TOTAL_WIDTH_M;

                double collectStartY = isRed
                        ? FieldLayout.FIELD_WIDTH_M - trenchEdge - TRENCH_Y_CLEARANCE_M
                        : trenchEdge + TRENCH_Y_CLEARANCE_M;
                double collectEndY   = isRed
                        ? collectStartY - COLLECT_SWEEP_Y_M
                        : collectStartY + COLLECT_SWEEP_Y_M;

                Pose2d collectStartPose = new Pose2d(neutralX, collectStartY, leftWallHeading);
                Pose2d collectEndPose   = new Pose2d(neutralX, collectEndY,   leftWallHeading);

                SmartDashboard.putString("TrenchOutpostAuto/Alliance", isRed ? "Red" : "Blue");

                // ── Tangents ──────────────────────────────────────────────────
                Rotation2d tOutbound    = tangentBetween(trenchCenter, trenchNeutralExitPose);
                Rotation2d tReturn      = tOutbound.rotateBy(Rotation2d.k180deg);
                Rotation2d tToCollect   = tangentBetween(trenchNeutralExitPose, collectStartPose);
                Rotation2d tFromCollect = tToCollect.rotateBy(Rotation2d.k180deg);
                Rotation2d tToOutpost   = tangentBetween(hubSideExitPose, outpostPose);

                // ── X-coordinate thresholds for trench sequencing ─────────────
                // Using X position avoids the unreliable 4-inch isInsideTrench window.
                final double neutralXThresh = trenchNeutralExitPose.getX();
                final double hubXThresh     = hubSideExitPose.getX();
                java.util.function.BooleanSupplier exitedOutbound = isRed
                        ? () -> drivetrain.getState().Pose.getX() < neutralXThresh
                        : () -> drivetrain.getState().Pose.getX() > neutralXThresh;
                java.util.function.BooleanSupplier enteredReturnTrench = isRed
                        ? () -> drivetrain.getState().Pose.getX() > neutralXThresh
                        : () -> drivetrain.getState().Pose.getX() < neutralXThresh;
                java.util.function.BooleanSupplier exitedReturn = isRed
                        ? () -> drivetrain.getState().Pose.getX() > hubXThresh
                        : () -> drivetrain.getState().Pose.getX() < hubXThresh;

                // ============================================================
                // 8-waypoint path (WP 0–7, t = 0.0–7.0):
                //
                //   WP0  trench centre          (outbound entry)
                //   WP1  neutral-side exit      (outbound)
                //   WP2  collect start
                //   WP3  collect end
                //   WP4  collect start          (return; same XY as WP2)
                //   WP5  neutral-side exit      (return)
                //   WP6  hub-side exit
                //   WP7  outpost                (GoalEndState v=0)
                //
                // RotationTargets:
                //   t=0.0  returnHeading     WP0 — opponent alliance wall
                //   t=1.0  returnHeading     WP1 — opponent alliance wall
                //   t=2.0  leftWallHeading   WP2 — collection wall
                //   t=3.0  leftWallHeading   WP3 — collection wall
                //   t=4.0  leftWallHeading   WP4 — collection wall
                //   t=5.0  allianceWallHeading  WP5 — own alliance wall
                //   t=6.0  allianceWallHeading  WP6 — own alliance wall
                //   t=6.5  outpostHeading       transitioning to outpost heading
                //
                // ConstraintZones:
                //   [0,1]  TRENCH_CONSTRAINTS  outbound trench
                //   [2,4]  COLLECT_CONSTRAINTS collect sweep
                //   [5,6]  TRENCH_CONSTRAINTS  return trench
                // ============================================================

                PathPlannerPath fullPath = new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(
                                // WP0 — trench centre (outbound entry)
                                new Pose2d(trenchCenter, tOutbound),
                                // WP1 — neutral-side exit (outbound)
                                new Pose2d(trenchNeutralExitPose.getTranslation(), tToCollect),
                                // WP2 — collect start
                                new Pose2d(collectStartPose.getTranslation(), collectTangent),
                                // WP3 — collect end (turnaround)
                                new Pose2d(collectEndPose.getTranslation(), collectTangentRev),
                                // WP4 — collect start (return; same XY as WP2)
                                new Pose2d(collectStartPose.getTranslation(), tFromCollect),
                                // WP5 — neutral-side exit (return)
                                new Pose2d(trenchNeutralExitPose.getTranslation(), tReturn),
                                // WP6 — hub-side exit
                                new Pose2d(hubSideExitPose.getTranslation(), tToOutpost),
                                // WP7 — outpost (GoalEndState)
                                new Pose2d(outpostPose.getTranslation(), tToOutpost)),
                        List.of(
                                new RotationTarget(0.0, returnHeading),              // WP0 — opponent alliance wall
                                new RotationTarget(1.0, returnHeading),              // WP1 — opponent alliance wall
                                new RotationTarget(2.0, leftWallHeading),            // WP2 — collection wall
                                new RotationTarget(3.0, leftWallHeading),            // WP3 — collection wall
                                new RotationTarget(4.0, leftWallHeading),            // WP4 — collection wall
                                new RotationTarget(5.0, allianceWallHeading),        // WP5 — own alliance wall
                                new RotationTarget(6.0, allianceWallHeading),        // WP6 — own alliance wall
                                new RotationTarget(6.5, outpostPose.getRotation())), // transition to outpost heading
                        List.of(),  // pointTowardsZones
                        List.of(
                                new ConstraintsZone(0.0, 1.0, TRENCH_CONSTRAINTS),
                                new ConstraintsZone(2.0, 4.0, COLLECT_CONSTRAINTS),
                                new ConstraintsZone(5.0, 6.0, TRENCH_CONSTRAINTS)),
                        List.of(),  // eventMarkers
                        CONSTRAINTS,
                        new IdealStartingState(TRENCH_CONSTRAINTS.maxVelocityMPS(), returnHeading),
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
                        intake.deploy();
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "0-PoseInit");
                    }),

                    Commands.runOnce(() ->
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "1-Running")),

                    Commands.deadline(
                        AutoBuilder.pathfindThenFollowPath(fullPath, CONSTRAINTS),

                        // Intake: roller runs in neutral zone, stops at return trench entry
                        Commands.sequence(
                            Commands.waitUntil(exitedOutbound::getAsBoolean),
                            Commands.runOnce(() ->
                                SmartDashboard.putString("TrenchOutpostAuto/Phase", "2-IntakeRunning")),
                            Commands.deadline(
                                Commands.waitUntil(enteredReturnTrench::getAsBoolean),
                                Commands.run(intake::runRoller, intake)
                            ),
                            Commands.runOnce(() -> {
                                intake.stopRoller();
                                SmartDashboard.putString("TrenchOutpostAuto/Phase", "6-RollerStopped");
                            })
                        ),

                        // Shoot on the move after return trench exit
                        Commands.sequence(
                            Commands.waitUntil(exitedReturn::getAsBoolean),
                            Commands.runOnce(() ->
                                SmartDashboard.putString("TrenchOutpostAuto/Phase", "7-ShootToOutpost")),
                            new ShootCommand(superstructure, vision, drivetrain, photonVision,
                                    intake, () -> false)
                                    .withTimeout(OutpostConstants.TRENCH_TO_OUTPOST_DRIVE_SHOOT_TIMEOUT_S)
                        )
                    ),

                    // At outpost: credit full chute and keep shooting.
                    Commands.runOnce(() -> {
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "8-ShootAtOutpost");
                        superstructure.setBallCount(OutpostConstants.OUTPOST_CHUTE_CAPACITY);
                    }),
                    Commands.deadline(
                        new ShootCommand(superstructure, vision, drivetrain, photonVision,
                                intake, () -> false)
                                .withTimeout(OutpostConstants.OUTPOST_RECEIVE_SHOOT_TIMEOUT_S),
                        Commands.run(() -> {}, drivetrain)
                    ),

                    Commands.runOnce(() ->
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "Done"))
                );
            },
            Set.of(drivetrain, superstructure, vision, intake)
        ).finallyDo(interrupted -> {
            intake.stopRoller();
            superstructure.requestState(RobotState.STOWED);
            SmartDashboard.putString("TrenchOutpostAuto/Phase",
                    interrupted ? "Interrupted" : "Complete");
        });
    }
}
