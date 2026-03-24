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
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * One-press teleop command that pathfinds through the nearest TRENCH.
 *
 * <h2>Direction logic</h2>
 * <ul>
 *   <li>Robot is on the <b>alliance side</b> (X &lt; field centre for Blue,
 *       X &gt; field centre for Red) → drives through the trench and stops at
 *       the <b>neutral-zone exit</b> (WP0 → WP1 of TrenchToOutpostAutoCommand).</li>
 *   <li>Robot is on the <b>neutral side</b> → drives through the trench and
 *       stops at the <b>alliance-zone exit</b>
 *       (WP6 → WP7 of TrenchToOutpostAutoCommand).</li>
 * </ul>
 *
 * <h2>Nearest trench</h2>
 * <p>Determined by the robot's Y position relative to field centre:
 * Y &lt; FIELD_WIDTH/2 → bottom-wall trench (index 0);
 * Y &ge; FIELD_WIDTH/2 → top-wall trench (index 1).</p>
 *
 * <h2>Heading convention</h2>
 * <p>Both directions face the opponent alliance wall (Blue 0°, Red 180°)
 * throughout the transit.</p>
 *
 * <p>{@link frc.robot.subsystems.TrenchTraversalManager} automatically manages
 * the {@code TRAVERSING_TRENCH} superstructure state during the transit.
 *
 * <p>Press once — the robot completes the transit and stops at the exit.
 * Any command that requires the drivetrain (e.g. the X-brake) will cancel it.
 */
public class TrenchTransitCommand {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
            3.0,
            2.5,
            Units.degreesToRadians(540),
            Units.degreesToRadians(720));

    private static final PathConstraints TRENCH_CONSTRAINTS = new PathConstraints(
            2.0,
            2.0,
            Units.degreesToRadians(360),
            Units.degreesToRadians(540));

    private TrenchTransitCommand() {}

    /** Compute the travel-direction tangent from one pose to another. */
    private static Rotation2d tangentBetween(Pose2d from, Pose2d to) {
        return new Rotation2d(to.getX() - from.getX(), to.getY() - from.getY());
    }

    /**
     * Creates the trench-transit command.
     *
     * @param drivetrain Swerve drivetrain (pathfinding + pose source).
     * @return Command that runs to completion when the robot reaches the exit pose.
     */
    public static Command create(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            boolean isRed = DriverStation.getAlliance()
                    .map(a -> a == DriverStation.Alliance.Red)
                    .orElse(false);

            Pose2d robotPose = drivetrain.getState().Pose;

            // ── Nearest trench (by Y) ─────────────────────────────────────────
            // Index 0 = bottom-wall trench; index 1 = top-wall trench.
            int trenchIdx = robotPose.getY() < FieldLayout.FIELD_WIDTH_M / 2.0 ? 0 : 1;

            // ── Through-poses (same XY as WP0/WP1/WP6/WP7 in TrenchToOutpostAutoCommand) ──
            //   hubSidePose    = WP0 / WP7 XY  (hub-side trench edge)
            //   neutralSidePose= WP1 / WP6 XY  (neutral-side trench edge)
            Pose2d hubSidePose = isRed
                    ? FieldLayout.RED_TRENCH_THROUGH_POSES[trenchIdx]
                    : FieldLayout.BLUE_TRENCH_THROUGH_POSES[trenchIdx];
            Pose2d neutralSidePose = isRed
                    ? FieldLayout.RED_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx]
                    : FieldLayout.BLUE_TRENCH_NEUTRAL_THROUGH_POSES[trenchIdx];

            // Both transit directions face the opponent alliance wall.
            //   Blue → 0°   (facing +X, toward Red wall)
            //   Red  → 180° (facing -X, toward Blue wall)
            Rotation2d outboundHeading = isRed
                    ? Rotation2d.k180deg
                    : Rotation2d.fromDegrees(0);

            // ── Zone detection ────────────────────────────────────────────────
            double fieldCenterX = FieldLayout.FIELD_LENGTH_M / 2.0;
            boolean inAllianceZone = isRed
                    ? robotPose.getX() > fieldCenterX
                    : robotPose.getX() < fieldCenterX;

            SmartDashboard.putString("TrenchTransit/Direction",
                    inAllianceZone ? "AllianceToNeutral(WP0→WP1)" : "NeutralToAlliance(WP6→WP7)");
            SmartDashboard.putString("TrenchTransit/Alliance", isRed ? "Red" : "Blue");
            SmartDashboard.putNumber("TrenchTransit/TrenchIdx", trenchIdx);

            if (inAllianceZone) {
                // ── Alliance → Neutral  (WP0 → WP1) ──────────────────────────
                // Pathfind to hub-side trench edge (WP0), then follow straight
                // through to the neutral-side exit (WP1).  Stop at WP1 facing
                // the opponent wall (outboundHeading).
                Rotation2d tangent = tangentBetween(hubSidePose, neutralSidePose);

                PathPlannerPath path = new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(
                                // WP0 — hub-side trench edge
                                new Pose2d(hubSidePose.getTranslation(),     tangent),
                                // WP1 — neutral-side trench exit
                                new Pose2d(neutralSidePose.getTranslation(), tangent)),
                        List.of(
                                new RotationTarget(0.3, outboundHeading),
                                new RotationTarget(0.7, outboundHeading)),
                        List.of(),  // pointTowardsZones
                        List.of(new ConstraintsZone(0.0, 1.0, TRENCH_CONSTRAINTS)),
                        List.of(),  // eventMarkers
                        CONSTRAINTS,
                        null,
                        new GoalEndState(0.0, outboundHeading),
                        false);
                path.preventFlipping = true;

                return AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS);

            } else {
                // ── Neutral → Alliance  (WP1 → WP0) ──────────────────────────
                // Pathfind to neutral-side trench edge, then follow straight
                // through to the hub-side exit.  Stop facing opponent wall.
                Rotation2d tangent = tangentBetween(neutralSidePose, hubSidePose);

                PathPlannerPath path = new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(neutralSidePose.getTranslation(), tangent),
                                new Pose2d(hubSidePose.getTranslation(),     tangent)),
                        List.of(
                                new RotationTarget(0.3, outboundHeading),
                                new RotationTarget(0.7, outboundHeading)),
                        List.of(),  // pointTowardsZones
                        List.of(new ConstraintsZone(0.0, 1.0, TRENCH_CONSTRAINTS)),
                        List.of(),  // eventMarkers
                        CONSTRAINTS,
                        null,
                        new GoalEndState(0.0, outboundHeading),
                        false);
                path.preventFlipping = true;

                return AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS);
            }

        }, Set.of(drivetrain));
    }
}
