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
 * Autonomous routine: continuous trench cycling — collect in the neutral zone,
 * return through the trench, shoot, then repeat.
 *
 * <h2>Cycle path — 7 waypoints (WP 0–6)</h2>
 * <ol start="0">
 *   <li>Trench centre (outbound entry)</li>
 *   <li>Neutral-zone trench exit (outbound)</li>
 *   <li>Collection start (robot faces wall)</li>
 *   <li>Collection end ({@value #COLLECT_SWEEP_Y_M} m sweep from WP2)</li>
 *   <li>Collection start again (return sweep; identical pose to WP2)</li>
 *   <li>Neutral-zone trench exit (return)</li>
 *   <li>Shoot position — hub-side trench exit in alliance zone ({@link GoalEndState} v=0)</li>
 * </ol>
 *
 * <h2>Cycle differences</h2>
 * <ul>
 *   <li>Cycle 1 — WP2 X: field-centre X ∓ (robotHalfWidth + safetyMargin);
 *       robot collects just shy of the field centre line.</li>
 *   <li>Cycle 2 — WP2 X: trench-centre X ± (trenchDepth/2 + robotHalfWidth + safetyMargin);
 *       robot starts collecting immediately past the trench neutral edge.</li>
 * </ul>
 */
public class TrenchCycleAutoCommand {

    /** Which trench to cycle through. */
    public enum Side { LEFT, RIGHT }

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

    /** Y distance swept during collection (hand-written; tune at practice). */
    private static final double COLLECT_SWEEP_Y_M = 2.0;

    /** Half-width of the robot used for cycle-2 trench-edge clearance. */
    private static final double ROBOT_HALF_WIDTH_M = Units.inchesToMeters(14.5);

    /** Extra safety margin used for both cycle-1 (field-centre offset) and cycle-2 (neutral-edge offset). */
    private static final double COLLECT_SAFETY_MARGIN_M = 0.15;

    /** Y clearance beyond the trench width to place the collect-start waypoint. */
    private static final double TRENCH_Y_CLEARANCE_M = 0.85;

    /** How long to shoot after each return through the trench. */
    private static final double SHOOT_TIMEOUT_S = 6.0;

    private TrenchCycleAutoCommand() {}

    private static Rotation2d tangentBetween(Translation2d from, Pose2d to) {
        return new Rotation2d(to.getX() - from.getX(), to.getY() - from.getY());
    }

    private static Rotation2d tangentBetween(Pose2d from, Pose2d to) {
        return new Rotation2d(to.getX() - from.getX(), to.getY() - from.getY());
    }

    /**
     * Creates the two-cycle trench autonomous command.
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
                boolean isRed   = DriverStation.getAlliance()
                        .map(a -> a == DriverStation.Alliance.Red)
                        .orElse(false);
                boolean isRight = (side == Side.RIGHT);

                // Trench index: RIGHT → Blue=0 (bottom wall), Red=1 (top wall)
                //               LEFT  → Blue=1 (top wall),   Red=0 (bottom wall)
                int trenchIdx = isRight ? (isRed ? 1 : 0) : (isRed ? 0 : 1);

                // Collection sweep direction: RIGHT+Blue or LEFT+Red → +Y; otherwise −Y.
                boolean sweepPositiveY = isRight != isRed;

                // Collect heading: robot faces the wall being swept toward.
                Rotation2d leftWallHeading   = Rotation2d.fromDegrees(sweepPositiveY ? 90 : -90);
                Rotation2d collectTangent    = Rotation2d.fromDegrees(sweepPositiveY ? 90 : -90);
                Rotation2d collectTangentRev = collectTangent.rotateBy(Rotation2d.k180deg);

                // Return heading: face opponent alliance wall (ready for next outbound trip).
                Rotation2d returnHeading = isRed
                        ? Rotation2d.k180deg
                        : Rotation2d.fromDegrees(0);

                // ── WP0: trench centre ────────────────────────────────────────
                Translation2d trenchCenter = isRed
                        ? FieldLayout.TRENCH_RED_CENTERS[trenchIdx]
                        : FieldLayout.TRENCH_BLUE_CENTERS[trenchIdx];

                // ── Through poses ─────────────────────────────────────────────
                Pose2d trenchNeutralExitPose = isRed
                        ? FieldLayout.RED_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx];

                Pose2d hubSideBase = isRed
                        ? FieldLayout.RED_TRENCH_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_THROUGH_POSES[trenchIdx];

                // WP6: shoot position — hub-side trench exit, return heading.
                Pose2d shootPose = new Pose2d(hubSideBase.getX(), hubSideBase.getY(), returnHeading);

                // Outbound tangent (trench centre → neutral exit) and its reverse.
                Rotation2d tOutbound = tangentBetween(trenchCenter, trenchNeutralExitPose);
                Rotation2d tReturn   = tOutbound.rotateBy(Rotation2d.k180deg);

                // ── Collection Y band (shared across cycles) ──────────────────
                double trenchEdge    = TrenchConstants.TRENCH_TOTAL_WIDTH_M;
                double collectStartY = sweepPositiveY
                        ? trenchEdge + TRENCH_Y_CLEARANCE_M
                        : FieldLayout.FIELD_WIDTH_M - trenchEdge - TRENCH_Y_CLEARANCE_M;
                double collectEndY   = sweepPositiveY
                        ? collectStartY + COLLECT_SWEEP_Y_M
                        : collectStartY - COLLECT_SWEEP_Y_M;

                double fieldCenterX = FieldLayout.FIELD_LENGTH_M / 2.0;

                // ── Cycle-1 collect start X ───────────────────────────────────
                // Robot centre just shy of the field centre line.
                double neutralX_c1 = isRed
                        ? fieldCenterX + ROBOT_HALF_WIDTH_M + COLLECT_SAFETY_MARGIN_M
                        : fieldCenterX - ROBOT_HALF_WIDTH_M - COLLECT_SAFETY_MARGIN_M;

                // ── Cycle-2 collect start X ───────────────────────────────────
                // Robot centre just past the neutral trench edge + robot half-width + safety.
                double cycle2offset = TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0
                        + ROBOT_HALF_WIDTH_M + COLLECT_SAFETY_MARGIN_M;
                double neutralX_c2  = isRed
                        ? trenchCenter.getX() - cycle2offset
                        : trenchCenter.getX() + cycle2offset;

                SmartDashboard.putString("TrenchCycleAuto/Alliance", isRed ? "Red" : "Blue");
                SmartDashboard.putString("TrenchCycleAuto/Side",     side.name());

                // ── Per-cycle builder ─────────────────────────────────────────
                // neutralX: X position of WP2 (collect start); varies by cycle.
                //
                // 7-waypoint path (WP 0–6, t = 0.0–6.0):
                //   WP0  trench centre          tangent = tOutbound
                //   WP1  neutral-side exit      tangent = tToCollect
                //   WP2  collect start          tangent = collectTangent
                //   WP3  collect end            tangent = collectTangentRev
                //   WP4  collect start (return) tangent = tFromCollect (−tToCollect)
                //   WP5  neutral-side exit      tangent = tReturn
                //   WP6  shoot position         tangent = tReturn
                //
                // RotationTargets:
                //   t=1.5  leftWallHeading   (rotate to face wall WP1→WP2)
                //   t=3.0  leftWallHeading   (hold at WP3 turnaround)
                //   t=4.5  returnHeading     (rotate back WP4→WP5)
                //   t=5.5  returnHeading     (hold through WP5→WP6)
                //
                // 7-waypoint path (WP 0–6, t = 0.0–6.0):
                //   WP0  trench centre          (outbound entry)
                //   WP1  neutral-side exit      (outbound)
                //   WP2  collect start
                //   WP3  collect end
                //   WP4  collect start          (return; same XY as WP2)
                //   WP5  neutral-side exit      (return)
                //   WP6  hub-side exit / shoot  (GoalEndState v=0)
                //
                // RotationTargets:
                //   t=0.0  returnHeading     WP0 — opponent alliance wall
                //   t=1.0  returnHeading     WP1 — opponent alliance wall
                //   t=2.0  leftWallHeading   WP2 — collection wall
                //   t=3.0  leftWallHeading   WP3 — collection wall
                //   t=4.0  leftWallHeading   WP4 — collection wall
                //   t=5.0  returnHeading     WP5 — opponent alliance wall
                //   t=5.5  returnHeading     WP6 — opponent alliance wall
                //
                // ConstraintZones:
                //   [0,1]  TRENCH_CONSTRAINTS  outbound trench
                //   [2,4]  COLLECT_CONSTRAINTS collect sweep
                //   [5,6]  TRENCH_CONSTRAINTS  return trench
                java.util.function.Function<Double, Command> buildCycle = (neutralX) -> {

                    Pose2d collectStartPose = new Pose2d(neutralX, collectStartY, leftWallHeading);
                    Pose2d collectEndPose   = new Pose2d(neutralX, collectEndY,   leftWallHeading);

                    Rotation2d tToCollect   = tangentBetween(trenchNeutralExitPose, collectStartPose);
                    Rotation2d tFromCollect = tToCollect.rotateBy(Rotation2d.k180deg);

                    PathPlannerPath cyclePath = new PathPlannerPath(
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
                                    // WP6 — hub-side exit / shoot (GoalEndState)
                                    new Pose2d(shootPose.getTranslation(), tReturn)),
                            List.of(
                                    new RotationTarget(0.0, returnHeading),   // WP0 — opponent alliance wall
                                    new RotationTarget(1.0, returnHeading),   // WP1 — opponent alliance wall
                                    new RotationTarget(2.0, leftWallHeading), // WP2 — collection wall
                                    new RotationTarget(3.0, leftWallHeading), // WP3 — collection wall
                                    new RotationTarget(4.0, leftWallHeading), // WP4 — collection wall
                                    new RotationTarget(5.0, returnHeading),   // WP5 — opponent alliance wall
                                    new RotationTarget(5.5, returnHeading)),  // WP6 — opponent alliance wall
                            List.of(),  // pointTowardsZones
                            List.of(
                                    new ConstraintsZone(0.0, 1.0, TRENCH_CONSTRAINTS),  // outbound trench
                                    new ConstraintsZone(2.0, 4.0, COLLECT_CONSTRAINTS), // collect sweep
                                    new ConstraintsZone(5.0, 6.0, TRENCH_CONSTRAINTS)), // return trench
                            List.of(),  // eventMarkers
                            CONSTRAINTS,
                            new IdealStartingState(TRENCH_CONSTRAINTS.maxVelocityMPS(), returnHeading),
                            new GoalEndState(0, returnHeading),
                            false);
                    cyclePath.preventFlipping = true;

                    // X-coordinate thresholds for reliable trench sequencing.
                    // Using X position avoids the unreliable 4-inch isInsideTrench window.
                    //   Blue: outbound exit = robot.x > neutralExit.x
                    //         return entry  = robot.x < neutralExit.x  (already past, collecting)
                    //         return exit   = robot.x < shoot.x         (back in alliance zone)
                    //   Red:  directions are flipped.
                    final double neutralXThresh = trenchNeutralExitPose.getX();
                    final double hubXThresh     = shootPose.getX();
                    java.util.function.BooleanSupplier exitedOutbound = isRed
                            ? () -> drivetrain.getState().Pose.getX() < neutralXThresh
                            : () -> drivetrain.getState().Pose.getX() > neutralXThresh;
                    java.util.function.BooleanSupplier enteredReturnTrench = isRed
                            ? () -> drivetrain.getState().Pose.getX() > neutralXThresh
                            : () -> drivetrain.getState().Pose.getX() < neutralXThresh;
                    java.util.function.BooleanSupplier exitedReturn = isRed
                            ? () -> drivetrain.getState().Pose.getX() > hubXThresh
                            : () -> drivetrain.getState().Pose.getX() < hubXThresh;

                    return Commands.sequence(

                        Commands.runOnce(() -> {
                            SmartDashboard.putString("TrenchCycleAuto/Phase", "Cycling");
                            SmartDashboard.putNumber("TrenchCycleAuto/NeutralX", neutralX);
                        }),

                        Commands.deadline(
                            // DEADLINE: intake management → wait for return trench exit → shoot.
                            // ShootCommand starts as soon as the robot clears the trench on
                            // the return trip; the path continues finishing to WP6 in parallel.
                            // When ShootCommand times out (SHOOT_TIMEOUT_S elapsed), the deadline
                            // ends and the path is interrupted (robot is at or near WP6 by then).
                            Commands.sequence(
                                // Wait for outbound trench exit (robot crosses neutral X threshold).
                                Commands.waitUntil(exitedOutbound::getAsBoolean),
                                Commands.runOnce(() ->
                                    SmartDashboard.putString("TrenchCycleAuto/Phase", "Intaking")),
                                // Run roller until robot re-enters the trench on the return trip.
                                Commands.deadline(
                                    Commands.waitUntil(enteredReturnTrench::getAsBoolean),
                                    Commands.run(intake::runRoller, intake)
                                ),
                                Commands.runOnce(() -> {
                                    intake.stopRoller();
                                    SmartDashboard.putString("TrenchCycleAuto/Phase", "RollerStopped");
                                }),
                                // Wait for return trench exit — robot is now in alliance zone.
                                Commands.waitUntil(exitedReturn::getAsBoolean),
                                // Start shooting immediately (path still finishing to WP6).
                                Commands.runOnce(() ->
                                    SmartDashboard.putString("TrenchCycleAuto/Phase", "Shooting")),
                                new ShootCommand(superstructure, vision, drivetrain, photonVision,
                                        intake, () -> false)
                                        .withTimeout(SHOOT_TIMEOUT_S)
                            ),

                            // PARALLEL: path WP0→WP6 runs to completion in the background.
                            // If path finishes before SHOOT_TIMEOUT_S elapses, the robot holds
                            // at WP6 and shooting continues until the deadline ends.
                            AutoBuilder.pathfindThenFollowPath(cyclePath, CONSTRAINTS)
                        )
                    );
                };

                return Commands.sequence(

                    // ── Vision pose init (once, before first cycle) ───────────
                    Commands.waitUntil(photonVision::hasPoseBeenCorrected)
                            .withTimeout(OutpostConstants.VISION_POSE_INIT_TIMEOUT_S),
                    Commands.runOnce(() -> {
                        Pose2d resetPose = photonVision.getLatestRawPose()
                                .orElseGet(() -> drivetrain.getState().Pose);
                        drivetrain.resetPose(resetPose);
                        intake.deploy();
                        SmartDashboard.putString("TrenchCycleAuto/Phase", "0-PoseInit");
                    }),

                    // ── Cycle 1 — deep sweep ──────────────────────────────────
                    Commands.defer(() -> buildCycle.apply(neutralX_c1),
                            Set.of(drivetrain, superstructure, vision, intake)),

                    // ── Cycle 2 — shallow sweep (just past trench neutral edge) ─
                    Commands.defer(() -> buildCycle.apply(neutralX_c2),
                            Set.of(drivetrain, superstructure, vision, intake))
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
