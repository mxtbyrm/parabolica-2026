package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.HubConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.TrenchConstants;

/**
 * Manages four corner-mounted PhotonVision cameras for global pose estimation.
 *
 * <h2>Overview</h2>
 * <p>Each camera sits at a corner of the chassis, yawed 45° toward the nearest
 * drivetrain side and tilted upward to see AprilTags above the TRENCH's low
 * clearance.  80° FOV lenses maximize coverage; combined with the four different
 * viewing angles, there are no persistent blind spots at most distances.
 *
 * <h2>Pose Estimation Strategy</h2>
 * <ol>
 *   <li>Try {@link PoseStrategy#MULTI_TAG_PNP_ON_RIO} when ≥2 tags are visible
 *       (lower std devs, unambiguous).</li>
 *   <li>Fall back to {@link PoseStrategy#LOWEST_AMBIGUITY} (single tag) when
 *       only one tag is visible; estimates with ambiguity above
 *       {@link PhotonVisionConstants#MAX_AMBIGUITY} are rejected.</li>
 *   <li>When no camera sees any tag the pose estimator continues on odometry
 *       only (no call to {@link CommandSwerveDrivetrain#addVisionMeasurement}).</li>
 * </ol>
 *
 * <p>Additional guard: if the gyro rotation rate exceeds
 * {@link PhotonVisionConstants#MAX_ROTATION_RATE_DEG_PER_S}, all camera
 * updates are skipped for that loop (motion blur + latency make them
 * unreliable at high angular velocity).
 *
 * <h2>Field Layout</h2>
 * <p>The field layout is built programmatically from the hub geometry in
 * {@link frc.robot.Constants}.  Positions are computed from
 * {@link FieldLayout#BLUE_HUB_CENTER} and
 * {@link HubConstants#HUB_BASE_WIDTH_M}.
 * <b>TODO: verify all tag X/Y positions and yaw angles against the physical
 * field before the first competition.</b>  TRENCH tag positions are marked with
 * TODO because their exact field coordinates have not yet been measured.
 *
 * <h2>Per-camera enable flags</h2>
 * <p>Set any of {@link PhotonVisionConstants#CAMERA_FL_ENABLED} …{@code _BR_ENABLED}
 * to {@code false} in Constants to bypass that camera entirely — useful when a
 * camera is physically missing or misconfigured.  The other three cameras
 * continue to provide pose measurements.
 *
 * <p>This subsystem is an <em>add-on</em> to {@link VisionSubsystem}: Limelight
 * targeting for HUB aiming and TRENCH proximity detection continues unchanged.
 * The four PhotonVision cameras provide only global pose correction.
 */
public class PhotonVisionSubsystem extends SubsystemBase {

    // =========================================================================
    // Constants
    // =========================================================================

    private static final int NUM_CAMERAS = 4;

    private static final String[] CAMERA_NAMES = {
        PhotonVisionConstants.CAMERA_FL_NAME,
        PhotonVisionConstants.CAMERA_FR_NAME,
        PhotonVisionConstants.CAMERA_BL_NAME,
        PhotonVisionConstants.CAMERA_BR_NAME,
    };

    private static final boolean[] CAMERA_ENABLED = {
        PhotonVisionConstants.CAMERA_FL_ENABLED,
        PhotonVisionConstants.CAMERA_FR_ENABLED,
        PhotonVisionConstants.CAMERA_BL_ENABLED,
        PhotonVisionConstants.CAMERA_BR_ENABLED,
    };

    private static final Transform3d[] ROBOT_TO_CAMERAS = {
        PhotonVisionConstants.ROBOT_TO_CAMERA_FL,
        PhotonVisionConstants.ROBOT_TO_CAMERA_FR,
        PhotonVisionConstants.ROBOT_TO_CAMERA_BL,
        PhotonVisionConstants.ROBOT_TO_CAMERA_BR,
    };

    private static final String[] CAMERA_LABELS = {"FL", "FR", "BL", "BR"};

    // =========================================================================
    // State
    // =========================================================================

    private final CommandSwerveDrivetrain m_drivetrain;
    private final PhotonCamera[]          m_cameras;
    private final PhotonPoseEstimator[]   m_estimators;
    private final AprilTagFieldLayout     m_fieldLayout;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Constructs the PhotonVisionSubsystem.
     *
     * @param drivetrain The swerve drivetrain whose pose estimator receives
     *                   vision measurements from every enabled camera.
     */
    public PhotonVisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain   = drivetrain;
        m_fieldLayout  = buildFieldLayout();
        m_cameras    = new PhotonCamera[NUM_CAMERAS];
        m_estimators = new PhotonPoseEstimator[NUM_CAMERAS];

        for (int i = 0; i < NUM_CAMERAS; i++) {
            m_cameras[i] = new PhotonCamera(CAMERA_NAMES[i]);

            m_estimators[i] = new PhotonPoseEstimator(m_fieldLayout, ROBOT_TO_CAMERAS[i]);
        }
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void periodic() {
        // Reject all vision updates when rotating too fast — motion blur and
        // timestamp latency errors make estimates unreliable at high angular velocity.
        double rotRateDegPerS = Math.abs(Units.radiansToDegrees(
                m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()));
        boolean tooFast = rotRateDegPerS > PhotonVisionConstants.MAX_ROTATION_RATE_DEG_PER_S;

        int activeCameras  = 0;
        int connectedCount = 0;

        for (int i = 0; i < NUM_CAMERAS; i++) {
            boolean connected = m_cameras[i].isConnected();
            SmartDashboard.putBoolean("PhotonVision/" + CAMERA_LABELS[i] + "/Connected", connected);

            if (!CAMERA_ENABLED[i]) continue;
            if (!connected)         continue;

            connectedCount++;

            for (var result : m_cameras[i].getAllUnreadResults()) {
                if (!result.hasTargets()) continue;

                if (tooFast) continue;

                // Try coprocessor multi-tag PnP first (returns empty if pipeline only saw 1 tag).
                var optPose = m_estimators[i].estimateCoprocMultiTagPose(result);
                if (optPose.isEmpty()) {
                    // Single-tag fallback: reject high-ambiguity estimates before solving.
                    double ambiguity = result.getBestTarget().getPoseAmbiguity();
                    if (ambiguity > PhotonVisionConstants.MAX_AMBIGUITY) {
                        SmartDashboard.putNumber("PhotonVision/" + CAMERA_LABELS[i] + "/RejectedAmbiguity",
                                ambiguity);
                        continue;
                    }
                    optPose = m_estimators[i].estimateLowestAmbiguityPose(result);
                }
                if (optPose.isEmpty()) continue;

                EstimatedRobotPose est = optPose.get();
                int numTags = est.targetsUsed.size();

                // Scale std devs with distance and tag count.
                // More tags → smaller sigma; farther tags → larger sigma.
                double avgDist = averageTagDistanceM(est);
                double distScale = avgDist * avgDist; // sigma grows quadratically with range
                double tagScale  = (numTags >= 2) ? PhotonVisionConstants.MULTI_TAG_STD_DEV_SCALE : 1.0;

                double xyStdDev    = PhotonVisionConstants.BASE_XY_STD_DEV_M    * distScale * tagScale;
                double thetaStdDev = PhotonVisionConstants.BASE_THETA_STD_DEV_RAD * distScale * tagScale;

                m_drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds,
                        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

                activeCameras++;

                SmartDashboard.putNumber("PhotonVision/" + CAMERA_LABELS[i] + "/NumTags", numTags);
                SmartDashboard.putNumber("PhotonVision/" + CAMERA_LABELS[i] + "/DistM",   avgDist);
                SmartDashboard.putNumber("PhotonVision/" + CAMERA_LABELS[i] + "/XYStdDev", xyStdDev);
            }
        }

        SmartDashboard.putNumber("PhotonVision/ActiveCameras",  activeCameras);
        SmartDashboard.putNumber("PhotonVision/Connected",      connectedCount);
        SmartDashboard.putBoolean("PhotonVision/TooFastSkip",   tooFast);
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /**
     * Returns the average straight-line distance from the camera pose to each
     * tag used in the estimate (meters).  Used to scale standard deviations.
     */
    private static double averageTagDistanceM(EstimatedRobotPose est) {
        if (est.targetsUsed.isEmpty()) return 4.0;
        double sum = 0;
        for (var target : est.targetsUsed) {
            var t = target.getBestCameraToTarget().getTranslation();
            sum += Math.sqrt(t.getX() * t.getX() + t.getY() * t.getY() + t.getZ() * t.getZ());
        }
        return sum / est.targetsUsed.size();
    }

    // =========================================================================
    // Field Layout Builder
    // =========================================================================

    /**
     * Builds an {@link AprilTagFieldLayout} from the hub and trench geometry
     * constants.  All tag 3D positions are derived from
     * {@link FieldLayout#BLUE_HUB_CENTER}, the HUB half-side length, and the
     * active AprilTag height.
     *
     * <p>Coordinate system: WPILib field origin at the Blue Alliance corner
     * (+X toward Red wall, +Y toward Blue's left wall, +Z up).
     *
     * <p>Tag face-normal convention: {@link Rotation3d#Rotation3d(double, double, double)}
     * is used as {@code (0, 0, yaw)} where {@code yaw} is the direction the tag
     * face points outward (toward the viewing camera).
     *
     * <p><b>All positions are approximate; verify against physical field.</b>
     */
    private static AprilTagFieldLayout buildFieldLayout() {
        List<AprilTag> tags = new ArrayList<>();

        // ── Blue HUB ─────────────────────────────────────────────────────────
        //
        // Blue HUB center:  (BX, BY) = (HUB_DIST_FROM_ALLIANCE_WALL, FIELD_WIDTH/2)
        // HUB half-side:    H = HUB_BASE_WIDTH / 2
        // AprilTag height:  Z = ACTIVE_HUB_APRILTAG_HEIGHT_M
        //
        // The TE-26306 face plate is 46.063 in wide.  Tag horizontal positions
        // are measured from the left edge of the face (as viewed from outside).
        // "Left" is defined from the perspective of someone standing outside
        // the HUB face and looking inward.
        //
        //   Side 1  (-X face, Blue-approach side, faces -X = yaw π)
        //   Side 2  (-Y face, right from Blue view,  faces -Y = yaw -π/2)
        //   Side 3  (+X face, far from Blue,         faces +X = yaw 0)
        //   Side 4  (+Y face, left from Blue view,   faces +Y = yaw +π/2)
        //
        // TODO: confirm face-to-side mapping against the physical field.

        double bx = FieldLayout.BLUE_HUB_CENTER.getX();
        double by = FieldLayout.BLUE_HUB_CENTER.getY();
        double h  = HubConstants.HUB_BASE_WIDTH_M / 2.0; // half-side ≈ 0.597 m
        double z  = HubConstants.ACTIVE_HUB_APRILTAG_HEIGHT_M;

        // Horizontal tag offsets from the left edge of each face plate.
        double ctr  = HubConstants.HUB_TAG_CENTER_OFFSET_M;  // 19.78 in → ≈ 0.502 m
        double left = HubConstants.HUB_TAG_LEFT_OFFSET_M;    //  5.78 in → ≈ 0.147 m
        double s4rt = HubConstants.HUB_TAG_SIDE4_RIGHT_OFFSET_M; // 40.283 in → ≈ 1.023 m

        // Side 1 — Blue-facing face (-X face of Blue HUB)
        // Viewed from -X, "left" = +Y.  Left edge at Y = by + h.
        addHubTag(tags, HubConstants.HUB_SIDE1_RIGHT_IDS[0], bx - h, (by + h) - ctr,  z, Math.PI);
        addHubTag(tags, HubConstants.HUB_SIDE1_LEFT_IDS[0],  bx - h, (by + h) - left, z, Math.PI);

        // Side 2 — Right face from Blue (-Y face of Blue HUB)
        // Viewed from -Y, "left" = -X.  Left edge at X = bx + h.
        addHubTag(tags, HubConstants.HUB_SIDE2_RIGHT_IDS[0], (bx + h) - ctr,  by - h, z, -Math.PI / 2);
        addHubTag(tags, HubConstants.HUB_SIDE2_LEFT_IDS[0],  (bx + h) - left, by - h, z, -Math.PI / 2);

        // Side 3 — Far face from Blue (+X face of Blue HUB)
        // Viewed from +X, "left" = -Y.  Left edge at Y = by - h.
        addHubTag(tags, HubConstants.HUB_SIDE3_RIGHT_IDS[0], bx + h, (by - h) + ctr,  z, 0);
        addHubTag(tags, HubConstants.HUB_SIDE3_LEFT_IDS[0],  bx + h, (by - h) + left, z, 0);

        // Side 4 — Left face from Blue (+Y face of Blue HUB)
        // Viewed from +Y, "left" = +X.  Left edge at X = bx + h.
        // NOTE: Side 4 layout is mirrored — centered tag is at the LEFT position,
        //       offset tag is at the RIGHT (from right edge = s4rt from left edge).
        addHubTag(tags, HubConstants.HUB_SIDE4_LEFT_IDS[0],  (bx + h) - ctr,  by + h, z, Math.PI / 2);
        addHubTag(tags, HubConstants.HUB_SIDE4_RIGHT_IDS[0], (bx + h) - s4rt, by + h, z, Math.PI / 2);

        // ── Red HUB ──────────────────────────────────────────────────────────
        //
        // Red HUB is the mirror of Blue HUB across the field X midpoint.
        //   rx = FIELD_LENGTH - bx, ry = by (same Y)
        //   Face normals flip X component: Side1→+X, Side2 and Side4 unchanged,
        //   Side3→-X.
        //
        // TODO: verify Red HUB tag positions against physical field.

        double rx = FieldLayout.RED_HUB_CENTER.getX();
        double ry = by;

        // Red Side 1 — Red-facing face (+X face of Red HUB, yaw = 0)
        addHubTag(tags, HubConstants.HUB_SIDE1_RIGHT_IDS[1], rx + h, (ry + h) - ctr,  z, 0);
        addHubTag(tags, HubConstants.HUB_SIDE1_LEFT_IDS[1],  rx + h, (ry + h) - left, z, 0);

        // Red Side 2 — mirrored from Blue Side 2 (-Y face, yaw = -π/2)
        addHubTag(tags, HubConstants.HUB_SIDE2_RIGHT_IDS[1], (rx - h) + ctr,  ry - h, z, -Math.PI / 2);
        addHubTag(tags, HubConstants.HUB_SIDE2_LEFT_IDS[1],  (rx - h) + left, ry - h, z, -Math.PI / 2);

        // Red Side 3 — far from Red (-X face of Red HUB, yaw = π)
        addHubTag(tags, HubConstants.HUB_SIDE3_RIGHT_IDS[1], rx - h, (ry - h) + ctr,  z, Math.PI);
        addHubTag(tags, HubConstants.HUB_SIDE3_LEFT_IDS[1],  rx - h, (ry - h) + left, z, Math.PI);

        // Red Side 4 — mirrored from Blue Side 4 (+Y face, yaw = +π/2)
        addHubTag(tags, HubConstants.HUB_SIDE4_LEFT_IDS[1],  (rx - h) + ctr,  ry + h, z, Math.PI / 2);
        addHubTag(tags, HubConstants.HUB_SIDE4_RIGHT_IDS[1], (rx - h) + s4rt, ry + h, z, Math.PI / 2);

        // ── TRENCH Tags ───────────────────────────────────────────────────────
        //
        // TRENCH tag positions are not yet measured.  Placeholder coordinates are
        // placed at the estimated TRENCH center positions from FieldLayout.
        // TODO: measure and replace with exact field positions before competition.

        double tz = TrenchConstants.ACTIVE_TRENCH_APRILTAG_HEIGHT_M;

        // Blue TRENCH tags — IDs 1, 6, 7, 12
        // Using estimated trench centers; actual positions TBD.
        addTrenchTagsBlue(tags, tz);

        // Red TRENCH tags — IDs 17, 22, 23, 28
        addTrenchTagsRed(tags, tz);

        AprilTagFieldLayout layout = new AprilTagFieldLayout(
                tags,
                FieldLayout.FIELD_LENGTH_M,
                FieldLayout.FIELD_WIDTH_M);

        // Always use the Blue alliance origin (WPILib standard).
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

        return layout;
    }

    /** Appends one HUB AprilTag to the list. */
    private static void addHubTag(List<AprilTag> tags, int id,
                                   double x, double y, double z, double yawRad) {
        tags.add(new AprilTag(id,
                new Pose3d(new Translation3d(x, y, z),
                           new Rotation3d(0, 0, yawRad))));
    }

    /**
     * Appends Blue TRENCH AprilTags with placeholder positions.
     * TODO: replace with measured field positions.
     */
    private static void addTrenchTagsBlue(List<AprilTag> tags, double z) {
        // Blue trench — bottom-wall structure (estimated)
        double bx0 = FieldLayout.TRENCH_BLUE_CENTERS[0].getX();
        double by0 = FieldLayout.TRENCH_BLUE_CENTERS[0].getY();
        tags.add(new AprilTag(1,  new Pose3d(new Translation3d(bx0 - 0.5, by0, z), new Rotation3d(0, 0, Math.PI))));
        tags.add(new AprilTag(6,  new Pose3d(new Translation3d(bx0 + 0.5, by0, z), new Rotation3d(0, 0, 0))));

        // Blue trench — top-wall structure (estimated)
        double bx1 = FieldLayout.TRENCH_BLUE_CENTERS[1].getX();
        double by1 = FieldLayout.TRENCH_BLUE_CENTERS[1].getY();
        tags.add(new AprilTag(7,  new Pose3d(new Translation3d(bx1 - 0.5, by1, z), new Rotation3d(0, 0, Math.PI))));
        tags.add(new AprilTag(12, new Pose3d(new Translation3d(bx1 + 0.5, by1, z), new Rotation3d(0, 0, 0))));
    }

    /**
     * Appends Red TRENCH AprilTags with placeholder positions.
     * TODO: replace with measured field positions.
     */
    private static void addTrenchTagsRed(List<AprilTag> tags, double z) {
        // Red trench — bottom-wall structure (estimated)
        double rx0 = FieldLayout.TRENCH_RED_CENTERS[0].getX();
        double ry0 = FieldLayout.TRENCH_RED_CENTERS[0].getY();
        tags.add(new AprilTag(17, new Pose3d(new Translation3d(rx0 + 0.5, ry0, z), new Rotation3d(0, 0, 0))));
        tags.add(new AprilTag(22, new Pose3d(new Translation3d(rx0 - 0.5, ry0, z), new Rotation3d(0, 0, Math.PI))));

        // Red trench — top-wall structure (estimated)
        double rx1 = FieldLayout.TRENCH_RED_CENTERS[1].getX();
        double ry1 = FieldLayout.TRENCH_RED_CENTERS[1].getY();
        tags.add(new AprilTag(23, new Pose3d(new Translation3d(rx1 + 0.5, ry1, z), new Rotation3d(0, 0, 0))));
        tags.add(new AprilTag(28, new Pose3d(new Translation3d(rx1 - 0.5, ry1, z), new Rotation3d(0, 0, Math.PI))));
    }
}
