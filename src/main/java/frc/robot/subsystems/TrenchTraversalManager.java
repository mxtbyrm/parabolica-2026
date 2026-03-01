package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.TrenchConstants;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Safety interlock manager for the TRENCH field element.
 *
 * <p>Each loop this subsystem:
 * <ol>
 *   <li>Reads the robot's current pose from the drivetrain.</li>
 *   <li>Checks whether the robot is inside a TRENCH bounding box via
 *       {@link #isInsideTrench(Pose2d)}.</li>
 *   <li>Checks whether the robot is within the approach threshold
 *       ({@link TrenchConstants#TRENCH_APPROACH_STOW_DISTANCE_M}) of any TRENCH.</li>
 *   <li>When approaching or inside the TRENCH, it requests
 *       {@link RobotState#TRAVERSING_TRENCH} from the Superstructure so that all
 *       overhead mechanisms are stowed.</li>
 *   <li>When the robot clears all TRENCHes, it returns the Superstructure to
 *       {@link RobotState#STOWED} (only if it was the one who requested TRAVERSING_TRENCH).</li>
 * </ol>
 *
 * <h2>Height Interlock</h2>
 * <p>The TRENCH clearance height is {@link TrenchConstants#TRENCH_CLEARANCE_HEIGHT_M}
 * (22.125 in).  The maximum safe robot height is {@link TrenchConstants#TRENCH_MAX_ROBOT_HEIGHT_M}
 * (21.125 in, including 1 in safety margin).  Because the robot's deployed height is
 * determined by mechanism state — not a live sensor — this manager enforces safety
 * by preventing mechanism extension whenever the robot is near or inside the TRENCH.
 *
 * <h2>Bounding Box Definition</h2>
 * <p>Each TRENCH structure is modeled as an axis-aligned rectangle centered at
 * a position in {@link FieldLayout#TRENCH_BLUE_CENTERS} or
 * {@link FieldLayout#TRENCH_RED_CENTERS}, with half-extents:
 * <pre>
 *   half_width = TRENCH_TOTAL_WIDTH_M / 2 + TRENCH_APPROACH_STOW_DISTANCE_M
 *   half_depth = TRENCH_TOTAL_DEPTH_M / 2 + TRENCH_APPROACH_STOW_DISTANCE_M
 * </pre>
 * The extra margin (the approach distance) ensures mechanisms are stowed before entry.
 */
public class TrenchTraversalManager extends SubsystemBase {

    // =========================================================================
    // Dependencies
    // =========================================================================

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Superstructure          m_superstructure;

    // =========================================================================
    // State
    // =========================================================================

    private boolean m_wasManagingTrench = false;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Constructs the TrenchTraversalManager.
     *
     * @param drivetrain     The swerve drivetrain (source of robot pose).
     * @param superstructure The superstructure state machine (target for TRAVERSING_TRENCH).
     */
    public TrenchTraversalManager(CommandSwerveDrivetrain drivetrain,
                                   Superstructure superstructure) {
        m_drivetrain      = drivetrain;
        m_superstructure  = superstructure;
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void periodic() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        boolean nearOrInside = isNearOrInsideTrench(robotPose);

        if (nearOrInside) {
            // Ensure mechanisms are stowed when approaching or inside any TRENCH.
            if (m_superstructure.getState() != RobotState.TRAVERSING_TRENCH) {
                m_superstructure.requestState(RobotState.TRAVERSING_TRENCH);
                m_wasManagingTrench = true;
            }
        } else if (m_wasManagingTrench) {
            // Robot has exited the TRENCH zone — return to STOWED.
            if (m_superstructure.getState() == RobotState.TRAVERSING_TRENCH) {
                m_superstructure.requestState(RobotState.STOWED);
            }
            m_wasManagingTrench = false;
        }

        SmartDashboard.putBoolean("Trench/NearOrInside", nearOrInside);
        SmartDashboard.putBoolean("Trench/Managing",     m_wasManagingTrench);
    }

    // =========================================================================
    // Public Utilities
    // =========================================================================

    /**
     * Returns {@code true} if the given pose is within any TRENCH structure boundary
     * (strict interior, without the approach margin).
     *
     * <p>Use this to determine if the robot is actively under the TRENCH cross-member,
     * as opposed to merely in the approach zone.
     *
     * @param pose The robot pose to test.
     * @return {@code true} if inside any TRENCH bounding box.
     */
    public static boolean isInsideTrench(Pose2d pose) {
        double halfW = TrenchConstants.TRENCH_TOTAL_WIDTH_M / 2.0;
        double halfD = TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0;

        for (Translation2d center : FieldLayout.TRENCH_BLUE_CENTERS) {
            if (isInBox(pose.getTranslation(), center, halfW, halfD)) return true;
        }
        for (Translation2d center : FieldLayout.TRENCH_RED_CENTERS) {
            if (isInBox(pose.getTranslation(), center, halfW, halfD)) return true;
        }
        return false;
    }

    /**
     * Returns {@code true} if the given pose is near or inside any TRENCH structure,
     * including the approach margin of
     * {@link TrenchConstants#TRENCH_APPROACH_STOW_DISTANCE_M}.
     *
     * @param pose The robot pose to test.
     * @return {@code true} if inside the expanded TRENCH safety zone.
     */
    public static boolean isNearOrInsideTrench(Pose2d pose) {
        double margin = TrenchConstants.TRENCH_APPROACH_STOW_DISTANCE_M;
        double halfW  = TrenchConstants.TRENCH_TOTAL_WIDTH_M  / 2.0 + margin;
        double halfD  = TrenchConstants.TRENCH_TOTAL_DEPTH_M  / 2.0 + margin;

        for (Translation2d center : FieldLayout.TRENCH_BLUE_CENTERS) {
            if (isInBox(pose.getTranslation(), center, halfW, halfD)) return true;
        }
        for (Translation2d center : FieldLayout.TRENCH_RED_CENTERS) {
            if (isInBox(pose.getTranslation(), center, halfW, halfD)) return true;
        }
        return false;
    }

    /**
     * Returns whether the robot is currently managing TRENCH traversal (i.e. the
     * robot is near/inside a TRENCH and TRAVERSING_TRENCH has been requested).
     *
     * @return {@code true} if currently traversing.
     */
    public boolean isManagingTrench() {
        return m_wasManagingTrench;
    }

    /**
     * Returns the nearest TRENCH distance for the given pose in meters, or
     * {@code Double.MAX_VALUE} if no TRENCH center is defined.
     * Useful for approach warnings in telemetry.
     *
     * @param pose The robot pose.
     * @return Euclidean distance to the nearest TRENCH center in meters.
     */
    public static double distanceToNearestTrench(Pose2d pose) {
        double minDist = Double.MAX_VALUE;
        Translation2d robotXY = pose.getTranslation();

        for (Translation2d center : FieldLayout.TRENCH_BLUE_CENTERS) {
            double d = robotXY.getDistance(center);
            if (d < minDist) minDist = d;
        }
        for (Translation2d center : FieldLayout.TRENCH_RED_CENTERS) {
            double d = robotXY.getDistance(center);
            if (d < minDist) minDist = d;
        }
        return minDist;
    }

    // =========================================================================
    // Private Helpers
    // =========================================================================

    /**
     * Axis-aligned bounding box check.
     *
     * @param point  Point to test.
     * @param center Box center.
     * @param halfW  Half-width (X-axis).
     * @param halfD  Half-depth (Y-axis).
     * @return {@code true} if the point is inside the box.
     */
    private static boolean isInBox(Translation2d point, Translation2d center,
                                    double halfW, double halfD) {
        return Math.abs(point.getX() - center.getX()) <= halfW
            && Math.abs(point.getY() - center.getY()) <= halfD;
    }
}
