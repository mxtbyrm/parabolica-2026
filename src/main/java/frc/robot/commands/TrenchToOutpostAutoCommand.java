package frc.robot.commands;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.EventMarker;
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
 * Autonomous routine: through trench → collect → return → shoot at hub → OUTPOST.
 *
 * <h2>Path — 8 waypoints (WP 0–7, t = 0–7)</h2>
 * <pre>
 * WP0 (t=0): trenchCenter          tOutbound
 * WP1 (t=1): trenchNeutralExitPose tToCollect         [TRENCH zone 0–1]
 * WP2 (t=2): collectStart          collectTangent
 * WP3 (t=3): collectEnd            collectTangentRev   ← CUSP — robot stops & reverses
 * WP4 (t=4): collectStart          tFromCollect        ← leftWallHeading, then rotates back
 * WP5 (t=5): trenchNeutralExitPose tReturn             [TRENCH zone 5–6]
 * WP6 (t=6): hubSideExitPose       tToOutpost
 * WP7 (t=7): outpostPose           tToOutpost          GoalEndState v=0
 * </pre>
 *
 * <h2>Event Markers</h2>
 * <ul>
 *   <li>t=1.5  StartRoller</li>
 *   <li>t=4.8  StopRoller — before trench re-entry</li>
 * </ul>
 *
 * <p>After docking at the outpost: shoot remaining balls, credit chute capacity,
 * receive and shoot outpost balls.
 */
public class TrenchToOutpostAutoCommand {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        4.0, 3.5,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    private static final double TRENCH_Y_CLEARANCE_M    = 0.85;
    private static final double ROBOT_HALF_WIDTH_M      = Units.inchesToMeters(14.5);
    private static final double COLLECT_SAFETY_MARGIN_M = 0.15;
    private static final double COLLECT_SWEEP_Y_M       = 2.0;

    private TrenchToOutpostAutoCommand() {}

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

                // Outbound heading: face opponent alliance wall.
                Rotation2d returnHeading = isRed ? Rotation2d.k180deg : Rotation2d.fromDegrees(0);

                // Return trench heading: face own alliance wall.
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

                // ── Field geometry ────────────────────────────────────────────
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
                // fieldCenterX gives |ΔX| ≈ 2.6 m from trenchNeutralExitPose,
                // keeping the WP1→WP2 Bezier arc well-formed (no S-curve/reversal).
                double fieldCenterX  = FieldLayout.FIELD_LENGTH_M / 2.0;
                double neutralX      = isRed
                        ? fieldCenterX + ROBOT_HALF_WIDTH_M + COLLECT_SAFETY_MARGIN_M
                        : fieldCenterX - ROBOT_HALF_WIDTH_M - COLLECT_SAFETY_MARGIN_M;
                double trenchEdge    = TrenchConstants.TRENCH_TOTAL_WIDTH_M;
                double collectStartY = isRed
                        ? FieldLayout.FIELD_WIDTH_M - trenchEdge - TRENCH_Y_CLEARANCE_M
                        : trenchEdge + TRENCH_Y_CLEARANCE_M;
                double collectEndY   = isRed
                        ? collectStartY - COLLECT_SWEEP_Y_M
                        : collectStartY + COLLECT_SWEEP_Y_M;

                Translation2d collectStart = new Translation2d(neutralX, collectStartY);
                Translation2d collectEnd   = new Translation2d(neutralX, collectEndY);

                // ── Tangents ──────────────────────────────────────────────────
                // Trench is axis-aligned: tOutbound = returnHeading (0° Blue / 180° Red).
                Rotation2d tOutbound = returnHeading;                               // 0° or 180°
                Rotation2d tReturn   = returnHeading.rotateBy(Rotation2d.k180deg);  // 180° or 0°

                // tToOutpost: outpost is NOT axis-aligned, compute from positions.
                Rotation2d tToOutpost = new Rotation2d(
                    outpostPose.getX() - hubSideExitPose.getX(),
                    outpostPose.getY() - hubSideExitPose.getY()
                );

                // This auto always uses the outpost-side trench:
                // Blue = bottom-wall (index 0, RIGHT side) → opponent wall facing
                // Red  = top-wall    (index 1, RIGHT side) → opponent wall facing
                // Both cases: startEndHeading = returnHeading (outbound = opponent wall).
                Rotation2d startEndHeading = returnHeading;

                // WP0: robot's actual pose at auto start
                Translation2d robotStart = drivetrain.getState().Pose.getTranslation();

                SmartDashboard.putString("TrenchOutpostAuto/Alliance", isRed ? "Red" : "Blue");

                // ── Pre-compute path at auto-start — no heavy math mid-match ──
                // 9 waypoints: WP0=robot, WP1=trenchCenter, WP2=neutralExit,
                //              WP3=collectStart, WP4=collectEnd(CUSP), WP5=collectStart,
                //              WP6=neutralExit, WP7=hubSideExit, WP8=outpost
                PathPlannerPath fullPath = new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(
                        // WP0 (t=0) — robot start
                        new Pose2d(robotStart,                              tOutbound),
                        // WP1 (t=1) — trench centre
                        new Pose2d(trenchCenter,                            tOutbound),
                        // WP2 (t=2) — neutral-side exit: straight trench ends, arc begins
                        new Pose2d(trenchNeutralExitPose.getTranslation(), tOutbound),
                        // WP3 (t=3) — collect start: arc ends, straight sweep begins
                        new Pose2d(collectStart,                           collectTangent),
                        // WP4 (t=4) — collect end: CUSP (intentional stop & reverse)
                        new Pose2d(collectEnd,                             collectTangentRev),
                        // WP5 (t=5) — collect start: return sweep ends, arc to trench begins
                        new Pose2d(collectStart,                           collectTangentRev),
                        // WP6 (t=6) — neutral-side exit (return trench entry)
                        new Pose2d(trenchNeutralExitPose.getTranslation(), tReturn),
                        // WP7 (t=7) — hub-side exit
                        new Pose2d(hubSideExitPose.getTranslation(),       tToOutpost),
                        // WP8 (t=8) — outpost dock (GoalEndState v=0)
                        new Pose2d(outpostPose.getTranslation(),           tToOutpost)
                    ),
                    List.of(
                        new RotationTarget(0.0, startEndHeading),           // WP0 — opponent wall
                        new RotationTarget(1.0, returnHeading),             // WP1 — face outbound
                        new RotationTarget(2.5, returnHeading),             // hold through trench exit
                        new RotationTarget(3.0, leftWallHeading),           // WP3 — face wall
                        new RotationTarget(4.0, leftWallHeading),           // WP4 — cusp
                        new RotationTarget(5.0, leftWallHeading),           // WP5 — end of return sweep
                        new RotationTarget(5.3, leftWallHeading),           // pin: prevent bleed
                        new RotationTarget(6.0, allianceWallHeading),       // WP6 — entering return trench
                        new RotationTarget(7.0, allianceWallHeading),       // WP7 — hub-side exit
                        new RotationTarget(8.0, outpostPose.getRotation())  // WP8 — outpost dock
                    ),
                    List.of(), // pointTowardsZones
                    List.of(), // constraintZones — CONSTRAINTS used everywhere
                    List.of(
                        new EventMarker("Deploy", 2.0,
                            Commands.runOnce(() -> intake.deploy())),
                        new EventMarker("StartRoller", 2.5,
                            Commands.runOnce(() -> intake.runRoller())),
                        new EventMarker("StopRoller", 5.8,
                            Commands.runOnce(() -> intake.stopRoller()))
                    ),
                    CONSTRAINTS,
                    new IdealStartingState(0, startEndHeading),
                    new GoalEndState(0, outpostPose.getRotation()),
                    false
                );
                fullPath.preventFlipping = true;

                return Commands.sequence(

                    // ── Vision pose init (immediate — no wait) ────────────────
                    Commands.runOnce(() -> {
                        Pose2d resetPose = photonVision.getLatestRawPose()
                                .orElseGet(() -> drivetrain.getState().Pose);
                        drivetrain.resetPose(resetPose);
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "0-PoseInit");
                    }),

                    // ── Drive: WP0→WP8, roller via EventMarkers ──────────────
                    Commands.runOnce(() ->
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "1-Driving")),
                    AutoBuilder.followPath(fullPath),

                    // ── Shoot trench-collect balls at outpost ─────────────────
                    Commands.runOnce(() ->
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "2-ShootPreOutpost")),
                    new ShootCommand(superstructure, vision, drivetrain, photonVision,
                            intake, () -> false)
                            .withTimeout(OutpostConstants.TRENCH_TO_OUTPOST_DRIVE_SHOOT_TIMEOUT_S),

                    // ── Receive outpost chute balls and shoot ─────────────────
                    Commands.runOnce(() -> {
                        SmartDashboard.putString("TrenchOutpostAuto/Phase", "3-ShootOutpost");
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
