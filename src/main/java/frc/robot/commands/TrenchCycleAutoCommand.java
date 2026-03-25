package frc.robot.commands;

import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
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
 * Autonomous routine: continuous trench cycling.
 *
 * <h2>Cycle path — 7 waypoints (WP 0–6, t = 0–6)</h2>
 * <pre>
 * WP0 (t=0): trenchCenter          tOutbound                          [TRENCH zone 0–1]
 * WP1 (t=1): trenchNeutralExitPose bisect(tOutbound, collectTangent)  ← corner bisector
 * WP2 (t=2): collectStart          collectTangent
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

    // Open-field: robot capable of 4.54 m/s.
    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        4.0, 3.5,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    // Trench transit — limited for mechanical clearance under the structure.
    private static final PathConstraints TRENCH_CONSTRAINTS = new PathConstraints(
        2.0, 2.0,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540)
    );

    // Collection sweep — slow for reliable ball pick-up.
    private static final PathConstraints COLLECT_CONSTRAINTS = new PathConstraints(
        1.5, 1.5,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540)
    );

    private static final double COLLECT_SWEEP_Y_M       = 2.0;
    private static final double ROBOT_HALF_WIDTH_M      = Units.inchesToMeters(14.5);
    private static final double COLLECT_SAFETY_MARGIN_M = 0.15;
    private static final double TRENCH_Y_CLEARANCE_M    = 0.85;

    private TrenchCycleAutoCommand() {}

    /** Builds the 7-waypoint cycle path. */
    private static PathPlannerPath buildCyclePath(
            Translation2d trenchCenter,
            Pose2d trenchNeutralExitPose,
            Pose2d shootPose,
            double neutralX,
            double collectStartY,
            double collectEndY,
            Rotation2d returnHeading,   // 0° Blue / 180° Red
            Rotation2d leftWallHeading, // 90° or −90°
            AtomicBoolean shootFlag,
            IntakeSubsystem intake) {

        Rotation2d tOutbound     = returnHeading;                               // 0° or 180°
        Rotation2d tReturn       = returnHeading.rotateBy(Rotation2d.k180deg);  // 180° or 0°
        Rotation2d collectTangent    = leftWallHeading;                         // 90° or −90°
        Rotation2d collectTangentRev = leftWallHeading.rotateBy(Rotation2d.k180deg); // −90° or 90°

        Translation2d collectStart   = new Translation2d(neutralX, collectStartY);
        Translation2d collectEnd     = new Translation2d(neutralX, collectEndY);

        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                // WP0 (t=0) — trench centre: straight segment starts here
                new Pose2d(trenchCenter,                            tOutbound),
                // WP1 (t=1) — neutral-side exit: straight trench ends, arc to collection begins
                new Pose2d(trenchNeutralExitPose.getTranslation(), tOutbound),
                // WP2 (t=2) — collect start: arc ends, straight sweep begins
                new Pose2d(collectStart,                           collectTangent),
                // WP3 (t=3) — collect end: CUSP (intentional stop & reverse)
                new Pose2d(collectEnd,                             collectTangentRev),
                // WP4 (t=4) — collect start: straight return sweep ends, arc to trench begins
                new Pose2d(collectStart,                           collectTangentRev),
                // WP5 (t=5) — neutral-side exit: arc ends, straight trench begins
                new Pose2d(trenchNeutralExitPose.getTranslation(), tReturn),
                // WP6 (t=6) — shoot pose (GoalEndState v=0)
                new Pose2d(shootPose.getTranslation(),             tReturn)
            ),
            List.of(
                new RotationTarget(0.0, returnHeading),   // WP0 — face opponent wall
                new RotationTarget(1.5, returnHeading),   // hold through trench exit
                new RotationTarget(2.0, leftWallHeading), // WP2 — face collection wall
                new RotationTarget(3.0, leftWallHeading), // WP3 — cusp, still facing wall
                new RotationTarget(4.0, leftWallHeading), // WP4 — end of return sweep
                new RotationTarget(4.3, leftWallHeading), // pin: prevent bleed before rotation
                new RotationTarget(5.0, returnHeading),   // WP5 — entering return trench
                new RotationTarget(6.0, returnHeading)    // WP6 — shoot pose
            ),
            List.of(), // pointTowardsZones
            List.of(
                new ConstraintsZone(0.0, 1.0, TRENCH_CONSTRAINTS),  // outbound under structure
                new ConstraintsZone(2.0, 4.0, COLLECT_CONSTRAINTS), // full out-and-back sweep
                new ConstraintsZone(5.0, 6.0, TRENCH_CONSTRAINTS)   // return under structure
            ),
            List.of(
                new EventMarker("StartRoller", 1.5,
                    Commands.runOnce(() -> intake.runRoller())),
                new EventMarker("StopRoller", 4.8,
                    Commands.runOnce(() -> intake.stopRoller())),
                new EventMarker("StartShoot", 5.6,
                    Commands.runOnce(() -> shootFlag.set(true)))
            ),
            CONSTRAINTS,
            new IdealStartingState(TRENCH_CONSTRAINTS.maxVelocityMPS(), returnHeading),
            new GoalEndState(0, returnHeading),
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

                // Cycle 1: collect at field centre (deep neutral zone)
                double neutralX_c1 = isRed
                        ? fieldCenterX + ROBOT_HALF_WIDTH_M + COLLECT_SAFETY_MARGIN_M
                        : fieldCenterX - ROBOT_HALF_WIDTH_M - COLLECT_SAFETY_MARGIN_M;

                // Cycle 2: collect just past the neutral trench edge
                double cycle2offset = TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0
                        + ROBOT_HALF_WIDTH_M + COLLECT_SAFETY_MARGIN_M;
                double neutralX_c2  = isRed
                        ? trenchCenter.getX() - cycle2offset
                        : trenchCenter.getX() + cycle2offset;

                // ── Per-cycle shoot flags (AtomicBoolean, set by EventMarker at t=5) ──
                // ShootCommand only requires superstructure + vision, so it runs in a
                // Commands.deadline alongside pathfindThenFollowPath (drivetrain) — no conflict.
                AtomicBoolean shootFlag1 = new AtomicBoolean(false);
                AtomicBoolean shootFlag2 = new AtomicBoolean(false);

                // ── Pre-compute both paths at auto-start — no heavy math mid-match ──
                PathPlannerPath cycle1Path = buildCyclePath(
                        trenchCenter, trenchNeutralExitPose, shootPose,
                        neutralX_c1, collectStartY, collectEndY,
                        returnHeading, leftWallHeading,
                        shootFlag1, intake);

                PathPlannerPath cycle2Path = buildCyclePath(
                        trenchCenter, trenchNeutralExitPose, shootPose,
                        neutralX_c2, collectStartY, collectEndY,
                        returnHeading, leftWallHeading,
                        shootFlag2, intake);

                SmartDashboard.putString("TrenchCycleAuto/Alliance", isRed ? "Red" : "Blue");
                SmartDashboard.putString("TrenchCycleAuto/Side",     side.name());

                return Commands.sequence(

                    // ── Vision pose init ──────────────────────────────────────
                    Commands.waitUntil(photonVision::hasPoseBeenCorrected)
                            .withTimeout(OutpostConstants.VISION_POSE_INIT_TIMEOUT_S),
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
                        SmartDashboard.putString("TrenchCycleAuto/Phase", "C1-Path");
                    }),
                    Commands.deadline(
                        // ShootCommand (superstructure+vision only) starts when
                        // EventMarker at t=5 sets shootFlag1. Runs 6 s while robot
                        // drives WP5→WP6 then holds at shootPose.
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
