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
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.Field;
import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.TrenchConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Autonomous routine: 2-cycle "trench bump" — robot drives outbound through trench,
 * collects in the neutral zone, returns through the hub bump, shoots, and repeats.
 * No stop between cycles; only the final shoot pose after cycle 2 stops the robot.
 *
 * <h2>Combined path — 17 waypoints (WP 0–16, t = 0–16)</h2>
 * <pre>
 * ── Cycle 1 (WP 0–8) ──────────────────────────────────────────────────────────
 * WP0  (t=0):  robot start           outboundHeading
 * WP1  (t=1):  trenchCenter          outboundHeading
 * WP2  (t=2):  trenchNeutralExit     outboundHeading  [Deploy EventMarker]
 * WP3  (t=3):  collectStart          wallDropHeading  [StartRoller]
 * WP4  (t=4):  collectEnd            collectHeading
 * WP5  (t=5):  bumpEntrance          collectHeading   [StopRoller; holonomic = 45° diagonal]
 * WP6  (t=6):  bumpExit              collectHeading   [StartShoot_C1]
 * WP7  (t=7):  arcPoint              arcMidHeading
 * WP8  (t=8):  shootPose             outboundHeading  [PASS-THROUGH — no stop, continues into C2]
 *
 * ── Cycle 2 (WP 8–16) ─────────────────────────────────────────────────────────
 * WP9  (t=9):  trenchCenter          outboundHeading  [StopShoot_C1 at t=9.8]
 * WP10 (t=10): trenchNeutralExit     outboundHeading  [Deploy EventMarker]
 * WP11 (t=11): collectStart          wallDropHeading  [StartRoller]
 * WP12 (t=12): collectEnd            collectHeading
 * WP13 (t=13): bumpEntrance          collectHeading   [StopRoller; holonomic = 45° diagonal]
 * WP14 (t=14): bumpExit              collectHeading   [StartShoot_C2]
 * WP15 (t=15): arcPoint              arcMidHeading
 * WP16 (t=16): shootPose             outboundHeading  [GoalEndState v=0 — FINAL STOP]
 * </pre>
 *
 * <h2>Bump geometry</h2>
 * <ul>
 *   <li>Entrance X: hub's neutral-zone-facing wall (hubCenterX ± HUB_BASE_WIDTH_M/2)</li>
 *   <li>Exit X: hub's alliance-facing wall (hubCenterX ∓ HUB_BASE_WIDTH_M/2)</li>
 *   <li>Bump Y: midpoint between nearest hub face Y (hubCenterY ± HUB_BASE_WIDTH_M/2)
 *       and trench center Y</li>
 *   <li>Bump entrance holonomic heading: 45° diagonal toward hub corner</li>
 * </ul>
 *
 * <h2>Shoot timing</h2>
 * <ul>
 *   <li><b>C1 shoot</b>: starts at bump exit (t=6.0), stops 0.8 past trench center (t=9.8)</li>
 *   <li><b>C2 shoot</b>: starts at bump exit (t=14.0), runs 6 s (robot at final stop)</li>
 * </ul>
 *
 * <h2>Intake deploy</h2>
 * <p>Deploy is triggered via EventMarker at the trench neutral exit (WP2, WP10) — never
 * before the trench to avoid catching on the trench structure.
 */
public class TrenchBumpAutoCommand {

    public enum Side { LEFT, RIGHT }

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        4.0, 3.5,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    /** Slower during ball collection — intake needs time to grab balls. */
    private static final PathConstraints COLLECT_CONSTRAINTS = new PathConstraints(
        1.5, 2.0,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540)
    );

    /** Reduced speed during shoot window — smoother motion for accurate shots. */
    private static final PathConstraints SHOOT_CONSTRAINTS = new PathConstraints(
        2.5, 2.5,
        Units.degreesToRadians(360),
        Units.degreesToRadians(540)
    );

    // Collect geometry
    private static final double COLLECT_SWEEP_Y_M  = 2.0;                        // Y sweep depth into field
    private static final double COLLECT_X_OFFSET_M = 0.2;                        // X offset from field-centre midline
    private static final double ROBOT_HALF_WIDTH_M = Units.inchesToMeters(14.5); // 29" square chassis


    private TrenchBumpAutoCommand() {}

    // =========================================================================

    private static PathPlannerPath buildCombinedPath(
            Translation2d startTranslation,
            Translation2d trenchCenter,
            Pose2d trenchNeutralExit,
            double collectStartX_c1,
            double collectStartX_c2,
            double collectStartY,
            double collectEndY,
            double bumpEntranceX,
            double bumpExitX,
            double bumpY,
            double arcMidX,
            double arcMidY,
            Pose2d shootPose,
            Rotation2d outboundHeading,
            Rotation2d collectHeading,
            Rotation2d wallDropHeading,
            Rotation2d arcMidHeading,
            Rotation2d bumpEntryHeading,
            Rotation2d startEndHeading,
            AtomicBoolean shootFlag1Start,
            AtomicBoolean shootFlag1Stop,
            AtomicBoolean shootFlag2Start,
            IntakeSubsystem intake) {

        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                // ── Cycle 1 ──────────────────────────────────────────────────
                // WP0 — robot start
                new Pose2d(startTranslation,                          outboundHeading),
                // WP1 — trench centre
                new Pose2d(trenchCenter,                              outboundHeading),
                // WP2 — neutral-side exit: straight through trench; deploy fires here
                new Pose2d(trenchNeutralExit.getTranslation(),        outboundHeading),
                // WP3 — collect start: 90° turn, begin Y sweep (C1 — shallow past midline)
                new Pose2d(collectStartX_c1, collectStartY,           wallDropHeading),
                // WP4 — collect end: sweep done, turn toward hub
                new Pose2d(collectStartX_c1, collectEndY,             wallDropHeading),
                // WP5 — bump entrance: diagonal approach to hub wall opening
                new Pose2d(bumpEntranceX, bumpY,                      collectHeading),
                // WP6 — bump exit: through hub structure, heading toward alliance
                new Pose2d(bumpExitX,     bumpY,                      collectHeading),
                // WP7 — arc midpoint: guides curve from bump Y down to trench Y
                new Pose2d(arcMidX,       arcMidY,                    arcMidHeading),
                // WP8 — shoot pose PASS-THROUGH: outbound tangent continues into cycle 2
                new Pose2d(shootPose.getTranslation(),                outboundHeading),

                // ── Cycle 2 ──────────────────────────────────────────────────
                // WP9 — trench centre (StopShoot_C1 fires 0.8 past here, t=9.8)
                new Pose2d(trenchCenter,                              outboundHeading),
                // WP10 — neutral-side exit: deploy fires here
                new Pose2d(trenchNeutralExit.getTranslation(),        outboundHeading),
                // WP11 — collect start (C2 — deeper, 3× robot-half past midline)
                new Pose2d(collectStartX_c2, collectStartY,           wallDropHeading),
                // WP12 — collect end
                new Pose2d(collectStartX_c2, collectEndY,             wallDropHeading),
                // WP13 — bump entrance
                new Pose2d(bumpEntranceX, bumpY,                      collectHeading),
                // WP14 — bump exit: StartShoot_C2 fires here
                new Pose2d(bumpExitX,     bumpY,                      collectHeading),
                // WP15 — arc midpoint
                new Pose2d(arcMidX,       arcMidY,                    arcMidHeading),
                // WP16 — shoot pose FINAL STOP (GoalEndState v=0)
                new Pose2d(shootPose.getTranslation(),                outboundHeading)
            ),
            List.of(
                // Holonomic headings — trench transit and shoot pose use startEndHeading;
                // collection sweep uses wallDropHeading; bump entrance = 45° diagonal.
                new RotationTarget(0.0,  startEndHeading),
                new RotationTarget(1.0,  startEndHeading),
                new RotationTarget(2.0,  startEndHeading),
                new RotationTarget(3.0,  wallDropHeading),   // collecting
                new RotationTarget(4.0,  wallDropHeading),   // sweep done
                new RotationTarget(5.0,  bumpEntryHeading),  // collectHeading ± 45°
                new RotationTarget(6.0,  bumpEntryHeading),  // same heading through bump exit
                new RotationTarget(7.0,  startEndHeading),   // arc mid faces own/opponent wall
                new RotationTarget(8.0,  startEndHeading),   // shoot pose pass-through
                new RotationTarget(9.0,  startEndHeading),
                new RotationTarget(10.0, startEndHeading),
                new RotationTarget(11.0, wallDropHeading),
                new RotationTarget(12.0, wallDropHeading),
                new RotationTarget(13.0, bumpEntryHeading),
                new RotationTarget(14.0, bumpEntryHeading),  // same heading through bump exit
                new RotationTarget(15.0, startEndHeading),   // arc mid faces own/opponent wall
                new RotationTarget(16.0, startEndHeading)
            ),
            List.of(), // pointTowardsZones
            List.of(
                // C1 collect sweep: WP3→WP4
                new ConstraintsZone(3.0, 4.0, COLLECT_CONSTRAINTS),
                // C1 shoot window: bump exit → 0.8 past trench center (t=6→9.8)
                new ConstraintsZone(6.0, 9.8, SHOOT_CONSTRAINTS),
                // C2 collect sweep: WP11→WP12
                new ConstraintsZone(11.0, 12.0, COLLECT_CONSTRAINTS),
                // C2 shoot window: bump exit → final stop (t=14→16)
                new ConstraintsZone(14.0, 16.0, SHOOT_CONSTRAINTS)
            ),
            List.of(
                // ── Cycle 1 ──────────────────────────────────────────────────
                // Deploy at neutral exit — safe to deploy only after clearing trench
                new EventMarker("Deploy_C1", 2.0,
                    Commands.runOnce(() -> intake.deploy())),
                new EventMarker("StartRoller_C1", 3.0,
                    Commands.runOnce(() -> intake.runRoller())),
                new EventMarker("StopRoller_C1", 4.8,
                    Commands.runOnce(() -> intake.stopRoller())),
                // Shoot starts at bump exit, stops 0.8 past next trench center
                new EventMarker("StartShoot_C1", 6.0,
                    Commands.runOnce(() -> shootFlag1Start.set(true))),
                new EventMarker("StopShoot_C1", 9.8,
                    Commands.runOnce(() -> shootFlag1Stop.set(true))),
                // ── Cycle 2 ──────────────────────────────────────────────────
                new EventMarker("Deploy_C2", 10.0,
                    Commands.runOnce(() -> intake.deploy())),
                new EventMarker("StartRoller_C2", 11.0,
                    Commands.runOnce(() -> intake.runRoller())),
                new EventMarker("StopRoller_C2", 12.8,
                    Commands.runOnce(() -> intake.stopRoller())),
                new EventMarker("StartShoot_C2", 14.0,
                    Commands.runOnce(() -> shootFlag2Start.set(true)))
            ),
            CONSTRAINTS,
            new IdealStartingState(0, startEndHeading),
            new GoalEndState(0, startEndHeading),
            false
        );
        path.preventFlipping = true;
        return path;
    }

    // =========================================================================

    /**
     * Creates the 2-cycle trench bump autonomous command.
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

                int trenchIdx       = isRight ? (isRed ? 1 : 0) : (isRed ? 0 : 1);
                boolean trenchOnTopWall = (trenchIdx == 1);

                // ── Headings ─────────────────────────────────────────────────────
                // outboundHeading: direction robot drives away from own alliance through trench
                Rotation2d outboundHeading = isRed ? Rotation2d.k180deg : Rotation2d.fromDegrees(0);
                // collectHeading: return direction (toward own alliance)
                Rotation2d collectHeading  = outboundHeading.rotateBy(Rotation2d.k180deg);
                // wallDropHeading: ±90°, perpendicular to trench, toward field centre
                Rotation2d wallDropHeading = trenchOnTopWall
                        ? Rotation2d.fromDegrees(-90) : Rotation2d.fromDegrees(90);
                // arcMidHeading: opposite of wallDrop — guides arc back to trench Y after bump
                Rotation2d arcMidHeading   = wallDropHeading.rotateBy(Rotation2d.k180deg);
                // bumpEntryHeading: +45° off the axis-aligned return heading for this wall side.
                //   Top-wall trench  → base = collectHeading (facing own alliance)
                //   Bottom-wall trench → base = outboundHeading (facing opponent alliance)
                //   Blue Left  (top,    col=180°): 180+45        = -135°
                //   Blue Right (bottom, out=0°):     0+45        =   45°
                //   Red Right  (top):               -135°+180   =   45°  (opposite of Blue Left)
                //   Red Left   (bottom):              45°+180   = -135°  (opposite of Blue Right)
                Rotation2d bumpEntryHeading = (trenchOnTopWall ? collectHeading : outboundHeading)
                        .rotateBy(Rotation2d.fromDegrees(45.0));
                if (isRed) bumpEntryHeading = bumpEntryHeading.rotateBy(Rotation2d.k180deg);
                // startEndHeading: LEFT = own alliance (inbound); RIGHT = opponent wall (outbound)
                Rotation2d startEndHeading = isRight ? outboundHeading : collectHeading;

                // ── Field positions ───────────────────────────────────────────────
                Translation2d trenchCenter = isRed
                        ? FieldLayout.TRENCH_RED_CENTERS[trenchIdx]
                        : FieldLayout.TRENCH_BLUE_CENTERS[trenchIdx];
                Pose2d trenchNeutralExit = isRed
                        ? FieldLayout.RED_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx];
                Pose2d hubSideBase = isRed
                        ? FieldLayout.RED_TRENCH_THROUGH_POSES[trenchIdx]
                        : FieldLayout.BLUE_TRENCH_THROUGH_POSES[trenchIdx];
                Pose2d shootPose = new Pose2d(hubSideBase.getX(), hubSideBase.getY(), outboundHeading);

                double trenchY = trenchCenter.getY();

                // sweepPosY=true → bottom trench (sweep toward higher Y);
                // sweepPosY=false → top trench (sweep toward lower Y). Matches TrenchCycleAutoCommand.
                boolean sweepPosY = !trenchOnTopWall;

                // ── Collect area ──────────────────────────────────────────────────────────────────
                // X anchored to field-centre midline:
                //   C1: ± ROBOT_HALF_WIDTH_M + offset  (shallow — robot just past midline)
                //   C2: ± ROBOT_HALF_WIDTH_M*3 + offset (deeper — 1.5 full robot widths past midline)
                //   Red (outbound -X): centre is on the positive-X side of midline
                //   Blue (outbound +X): centre is on the negative-X side of midline
                double fieldCenterX     = FieldLayout.FIELD_LENGTH_M / 2.0;
                double collectStartX_c1 = isRed
                        ? fieldCenterX + ROBOT_HALF_WIDTH_M + COLLECT_X_OFFSET_M
                        : fieldCenterX - ROBOT_HALF_WIDTH_M - COLLECT_X_OFFSET_M;
                double collectStartX_c2 = isRed
                        ? fieldCenterX + ROBOT_HALF_WIDTH_M * 3 + COLLECT_X_OFFSET_M
                        : fieldCenterX - ROBOT_HALF_WIDTH_M * 3 - COLLECT_X_OFFSET_M;
                // Y: robot collect-start row is exactly at the trench inner edge (no extra clearance)
                double collectStartY = sweepPosY
                        ? TrenchConstants.TRENCH_TOTAL_WIDTH_M
                        : FieldLayout.FIELD_WIDTH_M - TrenchConstants.TRENCH_TOTAL_WIDTH_M;
                double collectEndY = sweepPosY
                        ? collectStartY + COLLECT_SWEEP_Y_M
                        : collectStartY - COLLECT_SWEEP_Y_M;

                // ── Bump geometry ─────────────────────────────────────────────────────────────────
                double hubCenterX  = isRed
                        ? FieldLayout.RED_HUB_CENTER.getX()
                        : FieldLayout.BLUE_HUB_CENTER.getX();
                double hubCenterY  = FieldLayout.BLUE_HUB_CENTER.getY(); // FIELD_WIDTH_M/2
                double hubHalfSide = Field.HUB_BASE_WIDTH_M / 2.0;

                // Bump entrance X: hub neutral-zone-facing wall
                double bumpEntranceX = isRed
                        ? hubCenterX - hubHalfSide - Units.inchesToMeters(Math.sqrt(Math.pow(16, 2) + Math.pow(16, 2))) - COLLECT_X_OFFSET_M
                        : hubCenterX + hubHalfSide + Units.inchesToMeters(Math.sqrt(Math.pow(16, 2) + Math.pow(16, 2))) + COLLECT_X_OFFSET_M;
                // Bump exit X: hub alliance-facing wall
                double bumpExitX = isRed
                        ? hubCenterX + hubHalfSide + Units.inchesToMeters(Math.sqrt(Math.pow(16, 2) + Math.pow(16, 2))) + COLLECT_X_OFFSET_M
                        : hubCenterX - hubHalfSide - Units.inchesToMeters(Math.sqrt(Math.pow(16, 2) + Math.pow(16, 2))) - COLLECT_X_OFFSET_M;

                // Bump Y: midpoint between trench inner edge and nearest hub face Y.
                // Formula: (TRENCH_TOTAL_WIDTH_M + hubCenterY - hubHalfSide) / 2  [bottom trench]
                //          (FIELD_WIDTH_M - TRENCH_TOTAL_WIDTH_M + hubCenterY + hubHalfSide) / 2  [top]
                double trenchInnerEdge = sweepPosY
                        ? TrenchConstants.TRENCH_TOTAL_WIDTH_M
                        : FieldLayout.FIELD_WIDTH_M - TrenchConstants.TRENCH_TOTAL_WIDTH_M;
                double hubFaceY = sweepPosY
                        ? hubCenterY - hubHalfSide   // bottom trench → hub bottom face
                        : hubCenterY + hubHalfSide;  // top trench → hub top face
                double bumpY = (trenchInnerEdge + hubFaceY) / 2.0;

                // Arc midpoint X: centre between hub's alliance-facing wall and the alliance wall.
                //   Blue: (0 + hubCenterX − hubHalfSide) / 2
                //   Red:  (FIELD_LENGTH + hubCenterX + hubHalfSide) / 2
                double hubAllianceFaceX = isRed
                        ? hubCenterX + hubHalfSide
                        : hubCenterX - hubHalfSide;
                double allianceWallX = isRed ? FieldLayout.FIELD_LENGTH_M : 0.0;
                double arcMidX = (hubAllianceFaceX + allianceWallX) / 2.0;
                double arcMidY = (bumpY + trenchY) / 2.0;

                // ── Shoot flags ───────────────────────────────────────────────────
                AtomicBoolean shootFlag1Start = new AtomicBoolean(false);
                AtomicBoolean shootFlag1Stop  = new AtomicBoolean(false);
                AtomicBoolean shootFlag2Start = new AtomicBoolean(false);

                Translation2d robotStart = drivetrain.getState().Pose.getTranslation();

                PathPlannerPath combinedPath = buildCombinedPath(
                        robotStart, trenchCenter, trenchNeutralExit,
                        collectStartX_c1, collectStartX_c2, collectStartY, collectEndY,
                        bumpEntranceX, bumpExitX, bumpY,
                        arcMidX, arcMidY, shootPose,
                        outboundHeading, collectHeading, wallDropHeading,
                        arcMidHeading, bumpEntryHeading, startEndHeading,
                        shootFlag1Start, shootFlag1Stop, shootFlag2Start,
                        intake);

                SmartDashboard.putString("TrenchBumpAuto/Alliance", isRed ? "Red" : "Blue");
                SmartDashboard.putString("TrenchBumpAuto/Side",     side.name());

                return Commands.sequence(

                    // ── Vision pose init (immediate — no wait) ─────────────────
                    Commands.runOnce(() -> {
                        Pose2d resetPose = photonVision.getLatestRawPose()
                                .orElseGet(() -> drivetrain.getState().Pose);
                        drivetrain.resetPose(resetPose);
                        shootFlag1Start.set(false);
                        shootFlag1Stop.set(false);
                        shootFlag2Start.set(false);
                        SmartDashboard.putString("TrenchBumpAuto/Phase", "0-PoseInit");
                    }),

                    // ── Single deadline: path + sequential shoot phases ────────
                    // followPath ends at WP16 GoalEndState v=0.
                    // Shoot sequence:
                    //   C1 — fires at bump exit (t=6), gated off at t=9.8 (0.8 past trench centre)
                    //   C2 — fires at bump exit (t=14), runs 6 s while robot stops at shoot pose.
                    Commands.deadline(
                        Commands.sequence(
                            // C1 shoot: bump exit → past trench centre
                            Commands.waitUntil(shootFlag1Start::get),
                            Commands.runOnce(() ->
                                SmartDashboard.putString("TrenchBumpAuto/Phase", "C1-Shoot")),
                            Commands.deadline(
                                new ShootCommand(superstructure, vision, drivetrain,
                                        photonVision, intake, () -> false),
                                Commands.waitUntil(shootFlag1Stop::get)
                            ),
                            // C2 shoot: bump exit → final stop
                            Commands.waitUntil(shootFlag2Start::get),
                            Commands.runOnce(() ->
                                SmartDashboard.putString("TrenchBumpAuto/Phase", "C2-Shoot")),
                            new ShootCommand(superstructure, vision, drivetrain,
                                    photonVision, intake, () -> false).withTimeout(6.0)
                        ),
                        AutoBuilder.followPath(combinedPath)
                    )
                );
            },
            Set.of(drivetrain, superstructure, vision, intake)
        ).finallyDo(interrupted -> {
            intake.stopRoller();
            superstructure.requestState(RobotState.STOWED);
            SmartDashboard.putString("TrenchBumpAuto/Phase",
                    interrupted ? "Interrupted" : "Complete");
        });
    }
}
