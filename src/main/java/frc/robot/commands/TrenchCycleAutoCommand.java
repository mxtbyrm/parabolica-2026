package frc.robot.commands;

import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

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
import frc.robot.Constants.TrenchConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Autonomous routine: continuous trench cycling.
 *
 * <h2>Cycle path — 7 waypoints (WP 0–6, t = 0–6)</h2>
 * <pre>
 * WP0 (t=0): trenchCenter          tOutbound  [TRENCH zone 0–1, straight, zero curvature]
 * WP1 (t=1): trenchNeutralExitPose tOutbound  ← straight segment ends; 90° arc begins here
 * WP2 (t=2): collectStart          collectTangent  ← arc ends (long chord → low curvature)
 * WP3 (t=3): collectEnd            collectTangentRev  ← CUSP — robot stops &amp; reverses
 * WP4 (t=4): collectStart          bisect(collectTangentRev, tReturn) ← corner bisector
 * WP5 (t=5): trenchNeutralExitPose tReturn                            [TRENCH zone 5–6]
 * WP6 (t=6): shootPose             tReturn            GoalEndState v=0
 * </pre>
 *
 * <h2>Event Markers</h2>
 * <ul>
 *   <li>t=1.5  StartRoller</li>
 *   <li>t=4.8  StopRoller — before re-entering trench</li>
 *   <li>t=5.6  StartShoot — sets flag; ShootCommand starts in parallel deadline</li>
 * </ul>
 *
 * <h2>Shoot-while-driving</h2>
 * <p>ShootCommand requires only {@code superstructure} and {@code vision} — not
 * {@code drivetrain}. Therefore it can run in a {@code Commands.deadline} alongside
 * {@code pathfindThenFollowPath} without a subsystem conflict. When the StartShoot
 * EventMarker fires at WP5, the AtomicBoolean flag unblocks a {@code waitUntil},
 * ShootCommand starts and runs for 6 seconds while the robot drives WP5→WP6 and
 * then holds at shootPose.
 */
public class TrenchCycleAutoCommand {

    /** Which trench to cycle through. */
    public enum Side { LEFT, RIGHT }

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        4.0, 3.5,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    private static final double COLLECT_SWEEP_Y_M       = 2.0;
    private static final double ROBOT_HALF_WIDTH_M      = Units.inchesToMeters(14.5);
    private static final double COLLECT_SAFETY_MARGIN_M = 0.15;
    private static final double TRENCH_Y_CLEARANCE_M    = 0.85;

    private TrenchCycleAutoCommand() {}

    /**
     * Builds the 8-waypoint cycle path.
     *
     * <pre>
     * WP0 (t=0): robot start (current pose C1 / shootPose C2)  startEndHeading
     * WP1 (t=1): trenchCenter                                   tOutbound
     * WP2 (t=2): trenchNeutralExit                              tOutbound   [STRAIGHT — zero curvature]
     * WP3 (t=3): collectStart                                   collectTangent
     * WP4 (t=4): collectEnd                                     collectTangentRev  ← CUSP
     * WP5 (t=5): collectStart                                   collectTangentRev
     * WP6 (t=6): trenchNeutralExit                              tReturn
     * WP7 (t=7): shootPose                                      tReturn     [GoalEndState v=0]
     * </pre>
     *
     * @param startTranslation  WP0 — robot current pose (C1) or shootPose (C2).
     * @param startEndHeading   Holonomic heading at WP0 and WP7:
     *                          LEFT = own alliance wall; RIGHT = opponent alliance wall.
     */
    private static PathPlannerPath buildCyclePath(
            Translation2d startTranslation,
            Translation2d trenchCenter,
            Pose2d trenchNeutralExitPose,
            Pose2d shootPose,
            double neutralX,
            double collectStartY,
            double collectEndY,
            Rotation2d returnHeading,   // 0° Blue / 180° Red
            Rotation2d leftWallHeading, // 90° or −90°
            Rotation2d startEndHeading, // holonomic heading at start and shoot pose
            AtomicBoolean shootFlag,
            IntakeSubsystem intake) {

        Rotation2d tOutbound         = returnHeading;
        Rotation2d tReturn           = returnHeading.rotateBy(Rotation2d.k180deg);
        Rotation2d collectTangent    = leftWallHeading;
        Rotation2d collectTangentRev = leftWallHeading.rotateBy(Rotation2d.k180deg);

        Translation2d collectStart = new Translation2d(neutralX, collectStartY);
        Translation2d collectEnd   = new Translation2d(neutralX, collectEndY);

        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                // WP0 (t=0) — robot start; holonomic heading = startEndHeading (RotationTarget)
                new Pose2d(startTranslation,                        tOutbound),
                // WP1 (t=1) — trench centre: straight segment starts here
                new Pose2d(trenchCenter,                            tOutbound),
                // WP2 (t=2) — neutral-side exit: STRAIGHT ends here; 90° arc to collection begins
                new Pose2d(trenchNeutralExitPose.getTranslation(),  tOutbound),
                // WP3 (t=3) — collect start: arc ends, straight sweep begins
                new Pose2d(collectStart,                            collectTangent),
                // WP4 (t=4) — collect end: CUSP (intentional stop & reverse)
                new Pose2d(collectEnd,                              collectTangentRev),
                // WP5 (t=5) — collect start: return sweep ends, arc to trench begins
                new Pose2d(collectStart,                            collectTangentRev),
                // WP6 (t=6) — neutral-side exit: arc ends, straight trench begins
                new Pose2d(trenchNeutralExitPose.getTranslation(),  tReturn),
                // WP7 (t=7) — shoot pose (GoalEndState v=0)
                new Pose2d(shootPose.getTranslation(),              tReturn)
            ),
            List.of(
                new RotationTarget(0.0, startEndHeading), // WP0
                new RotationTarget(1.0, startEndHeading), // WP1 — through trench
                new RotationTarget(2.5, startEndHeading), // hold through trench exit
                new RotationTarget(3.0, leftWallHeading), // WP3 — face collection wall
                new RotationTarget(4.0, leftWallHeading), // WP4 — cusp, still facing wall
                new RotationTarget(5.0, leftWallHeading), // WP5 — end of return sweep
                new RotationTarget(5.3, leftWallHeading), // pin: prevent bleed before rotation
                new RotationTarget(6.0, startEndHeading), // WP6 — entering return trench
                new RotationTarget(7.0, startEndHeading)  // WP7 — shoot pose
            ),
            List.of(), // pointTowardsZones
            List.of(), // constraintZones — CONSTRAINTS used everywhere
            List.of(
                new EventMarker("Deploy", 2.0,
                    Commands.runOnce(() -> intake.deploy())),
                new EventMarker("StartRoller", 2.5,
                    Commands.runOnce(() -> intake.runRoller())),
                new EventMarker("StopRoller", 5.8,
                    Commands.runOnce(() -> intake.stopRoller())),
                new EventMarker("StartShoot", 6.6,
                    Commands.runOnce(() -> shootFlag.set(true)))
            ),
            CONSTRAINTS,
            new IdealStartingState(0, startEndHeading),
            new GoalEndState(0, startEndHeading),
            false
        );
        path.preventFlipping = true;
        return path;
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

                int trenchIdx      = isRight ? (isRed ? 1 : 0) : (isRed ? 0 : 1);
                boolean sweepPosY  = isRight != isRed;

                Rotation2d leftWallHeading = Rotation2d.fromDegrees(sweepPosY ? 90 : -90);
                Rotation2d returnHeading   = isRed ? Rotation2d.k180deg : Rotation2d.fromDegrees(0);

                // startEndHeading: holonomic facing at cycle start (WP0) and shoot pose (WP7).
                // LEFT  → own alliance wall (inbound = face toward hub)
                // RIGHT → opponent alliance wall (outbound = face away from hub)
                Rotation2d startEndHeading = isRight
                        ? returnHeading
                        : returnHeading.rotateBy(Rotation2d.k180deg);

                Translation2d trenchCenter = isRed
                        ? FieldLayout.TRENCH_RED_CENTERS[trenchIdx]
                        : FieldLayout.TRENCH_BLUE_CENTERS[trenchIdx];

                Pose2d trenchNeutralExitPose = isRed
                        ? FieldLayout.RED_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx];

                Pose2d hubSideBase = isRed
                        ? FieldLayout.RED_TRENCH_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_THROUGH_POSES[trenchIdx];

                Pose2d shootPose = new Pose2d(hubSideBase.getX(), hubSideBase.getY(), returnHeading);

                double trenchEdge    = TrenchConstants.TRENCH_TOTAL_WIDTH_M;
                double collectStartY = sweepPosY
                        ? trenchEdge + TRENCH_Y_CLEARANCE_M
                        : FieldLayout.FIELD_WIDTH_M - trenchEdge - TRENCH_Y_CLEARANCE_M;
                double collectEndY   = sweepPosY
                        ? collectStartY + COLLECT_SWEEP_Y_M
                        : collectStartY - COLLECT_SWEEP_Y_M;

                double fieldCenterX = FieldLayout.FIELD_LENGTH_M / 2.0;

                // Cycle 1: collect at field centre (deep neutral zone).
                // Using fieldCenterX gives |ΔX| ≈ 2.6 m from trenchNeutralExitPose,
                // which keeps the WP1→WP2 Bezier arc well-formed (no S-curve).
                double neutralX_c1 = isRed
                        ? fieldCenterX + ROBOT_HALF_WIDTH_M + COLLECT_SAFETY_MARGIN_M
                        : fieldCenterX - ROBOT_HALF_WIDTH_M - COLLECT_SAFETY_MARGIN_M;

                // Cycle 2: collect 1.5 m past the neutral trench exit.
                // WP1→WP2 chord ≈ 2.4 m → radius ≈ 1.3 m → curvature-limited v_max ≈ 1.6 m/s
                // (a gentle corner, not a stop). Minimum safe offset is 0.65 m to prevent
                // a Bezier S-curve reversal; 1.5 m gives comfortable margin.
                double neutralX_c2 = isRed
                        ? trenchNeutralExitPose.getX() - 1.5
                        : trenchNeutralExitPose.getX() + 1.5;

                // ── Per-cycle shoot flags ─────────────────────────────────────
                AtomicBoolean shootFlag1 = new AtomicBoolean(false);
                AtomicBoolean shootFlag2 = new AtomicBoolean(false);

                // WP0 translations: C1 = robot's actual pose; C2 = shootPose (robot is there after C1).
                Translation2d robotStart = drivetrain.getState().Pose.getTranslation();

                // ── Pre-compute both paths at auto-start — no heavy math mid-match ──
                PathPlannerPath cycle1Path = buildCyclePath(
                        robotStart, trenchCenter, trenchNeutralExitPose, shootPose,
                        neutralX_c1, collectStartY, collectEndY,
                        returnHeading, leftWallHeading, startEndHeading,
                        shootFlag1, intake);

                PathPlannerPath cycle2Path = buildCyclePath(
                        shootPose.getTranslation(), trenchCenter, trenchNeutralExitPose, shootPose,
                        neutralX_c2, collectStartY, collectEndY,
                        returnHeading, leftWallHeading, startEndHeading,
                        shootFlag2, intake);

                SmartDashboard.putString("TrenchCycleAuto/Alliance", isRed ? "Red" : "Blue");
                SmartDashboard.putString("TrenchCycleAuto/Side",     side.name());

                return Commands.sequence(

                    // ── Vision pose init (immediate — no wait) ────────────────
                    // Pose is already good from disabled-period vision corrections.
                    // Waiting here cost 3–4 s at auto start.
                    Commands.runOnce(() -> {
                        Pose2d resetPose = photonVision.getLatestRawPose()
                                .orElseGet(() -> drivetrain.getState().Pose);
                        drivetrain.resetPose(resetPose);
                        SmartDashboard.putString("TrenchCycleAuto/Phase", "0-PoseInit");
                    }),

                    // ── Cycle 1 ───────────────────────────────────────────────
                    Commands.runOnce(() -> {
                        shootFlag1.set(false);
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
                        AutoBuilder.followPath(cycle1Path)
                    ),

                    // ── Cycle 2 ───────────────────────────────────────────────
                    Commands.runOnce(() -> {
                        shootFlag2.set(false);
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
                        AutoBuilder.followPath(cycle2Path)
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
