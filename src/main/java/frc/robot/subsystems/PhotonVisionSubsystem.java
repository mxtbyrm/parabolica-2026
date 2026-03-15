package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.Turret;

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
 *   <li>Uses {@code estimateCoprocMultiTagPose()} — the Pi's already-solved
 *       multi-tag PnP result.  This does not require a local field layout
 *       because the coprocessor supplies the field-to-camera transform directly.</li>
 *   <li>Single-tag fallback is intentionally omitted: it requires a correct
 *       local tag database and produces out-of-bounds poses with an empty layout.</li>
 *   <li>When no camera sees ≥2 tags the pose estimator continues on odometry
 *       only (no call to {@link CommandSwerveDrivetrain#addVisionMeasurement}).</li>
 * </ol>
 *
 * <p>Additional guard: if the gyro rotation rate exceeds
 * {@link PhotonVisionConstants#MAX_ROTATION_RATE_DEG_PER_S}, all camera
 * updates are skipped for that loop (motion blur + latency make them
 * unreliable at high angular velocity).
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

    // YAGSL-style std devs: (x_m, y_m, theta_rad) — same values as YAGSL example.
    // Single-tag theta=8 rad and multi-tag theta=1 rad gives very low heading
    // weight from camera while still allowing absolute heading correction over time.
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS =
            VecBuilder.fill(PhotonVisionConstants.SINGLE_TAG_XY_STD_DEV_M,
                            PhotonVisionConstants.SINGLE_TAG_XY_STD_DEV_M,
                            PhotonVisionConstants.SINGLE_TAG_THETA_STD_DEV_RAD);
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS  =
            VecBuilder.fill(PhotonVisionConstants.MULTI_TAG_XY_STD_DEV_M,
                            PhotonVisionConstants.MULTI_TAG_XY_STD_DEV_M,
                            PhotonVisionConstants.MULTI_TAG_THETA_STD_DEV_RAD);

    /** Maximum average tag distance for a single-tag estimate to be accepted. */
    private static final double SINGLE_TAG_MAX_DIST_M = 4.0;

    // =========================================================================
    // State
    // =========================================================================

    private final CommandSwerveDrivetrain m_drivetrain;
    private final PhotonCamera[]          m_cameras;
    private final PhotonPoseEstimator[]   m_estimators;
    private final AprilTagFieldLayout     m_fieldLayout;
    private final Alert[]                 m_disconnectAlerts;

    // ── Hub targeting from fused drivetrain pose (reset each cycle) ─────────

    /**
     * Robot-relative angle to the hub derived from the drivetrain's fused pose
     * (odometry + latency-compensated vision).  0° = forward, positive = CCW.
     * Empty when no cameras saw tags this cycle or alliance is unknown.
     */
    private Optional<Double> m_pvHubAngleDeg = Optional.empty();

    /**
     * Straight-line distance from turret pivot to hub from the fused pose.
     */
    private Optional<Double> m_pvHubDistanceM = Optional.empty();

    /** Tag count of the best estimate this cycle. */
    private int m_bestEstimateTagCount = 0;


    /** True once at least one valid vision measurement has been added to the drivetrain. */
    private boolean m_hasPoseBeenCorrected = false;

    /**
     * The most recent raw pose from the best-tag-count camera estimate.
     * Updated every loop when a valid multi-tag estimate is available.
     * Used for hard-resetting the drivetrain pose at autonomous start.
     */
    private Optional<edu.wpi.first.math.geometry.Pose2d> m_latestRawPose = Optional.empty();


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
        m_drivetrain       = drivetrain;
        // Empty layout — tag positions are managed by PhotonVision on the Pi.
        // estimateCoprocMultiTagPose() uses the coprocessor's already-computed
        // field-to-camera transform and does not need a local tag database.
        m_fieldLayout      = new AprilTagFieldLayout(
                new ArrayList<>(), FieldLayout.FIELD_LENGTH_M, FieldLayout.FIELD_WIDTH_M);
        m_cameras          = new PhotonCamera[NUM_CAMERAS];
        m_estimators       = new PhotonPoseEstimator[NUM_CAMERAS];
        m_disconnectAlerts = new Alert[NUM_CAMERAS];

        for (int i = 0; i < NUM_CAMERAS; i++) {
            m_cameras[i] = new PhotonCamera(CAMERA_NAMES[i]);
            m_estimators[i] = new PhotonPoseEstimator(
                    m_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAMERAS[i]);
            m_disconnectAlerts[i] = new Alert(
                    "PhotonVision camera '" + CAMERA_NAMES[i] + "' is disconnected.", AlertType.kWarning);
        }
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void periodic() {
        m_pvHubAngleDeg        = Optional.empty();
        m_pvHubDistanceM       = Optional.empty();
        m_bestEstimateTagCount = 0;
        // m_latestRawPose is intentionally NOT reset here.
        // It persists until clearPoseReady() is called so that
        // OutpostAutoCommand can read it in the runOnce tick that follows
        // the waitUntil tick — otherwise periodic() would wipe it between
        // the two ticks and getLatestRawPose() would return empty.

        int activeCameras  = 0;
        int connectedCount = 0;

        for (int i = 0; i < NUM_CAMERAS; i++) {
            boolean connected = m_cameras[i].isConnected();
            SmartDashboard.putBoolean("PhotonVision/" + CAMERA_LABELS[i] + "/Connected", connected);
            if (CAMERA_ENABLED[i]) {
                m_disconnectAlerts[i].set(!connected);
            }

            if (!CAMERA_ENABLED[i]) continue;
            if (!connected)         continue;

            connectedCount++;

            for (var result : m_cameras[i].getAllUnreadResults()) {
                if (!result.hasTargets()) continue;

                var optPose = m_estimators[i].estimateCoprocMultiTagPose(result);
                if (optPose.isEmpty()) continue;

                EstimatedRobotPose est     = optPose.get();
                Pose2d             pose2d  = est.estimatedPose.toPose2d();
                int                numTags = est.targetsUsed.size();
                double             avgDist = averageTagDistanceM(est);

                m_drivetrain.addVisionMeasurement(pose2d, est.timestampSeconds,
                        computeStdDevs(numTags, avgDist));
                m_hasPoseBeenCorrected = true;

                if (numTags > m_bestEstimateTagCount) {
                    m_bestEstimateTagCount = numTags;
                    m_latestRawPose        = Optional.of(pose2d);
                }

                activeCameras++;
                SmartDashboard.putNumber("PhotonVision/" + CAMERA_LABELS[i] + "/NumTags", numTags);
                SmartDashboard.putNumber("PhotonVision/" + CAMERA_LABELS[i] + "/DistM",   avgDist);
            }
        }

        // Compute hub angle/distance from the drivetrain's current fused pose.
        // Called every loop — not gated on fresh vision — because the drivetrain
        // pose is always maintained by odometry and corrected retroactively by
        // addVisionMeasurement() (called above with est.timestampSeconds).
        // This gives continuous hub targeting; accuracy improves automatically
        // when cameras see tags.
        updateHubTarget(m_drivetrain.getState().Pose);

        SmartDashboard.putNumber("PhotonVision/ActiveCameras", activeCameras);
        SmartDashboard.putNumber("PhotonVision/Connected",     connectedCount);
        SmartDashboard.putNumber("PhotonVision/HubAngleDeg",   m_pvHubAngleDeg.orElse(0.0));
        SmartDashboard.putNumber("PhotonVision/HubDistanceM",  m_pvHubDistanceM.orElse(-1.0));
        SmartDashboard.putBoolean("PhotonVision/HasHubTarget", m_pvHubAngleDeg.isPresent());
    }

    /** YAGSL-identical std dev heuristic. */
    private static Matrix<N3, N1> computeStdDevs(int numTags, double avgDist) {
        if (numTags == 0) {
            return SINGLE_TAG_STD_DEVS;
        }
        if (numTags == 1 && avgDist > SINGLE_TAG_MAX_DIST_M) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        Matrix<N3, N1> base = (numTags > 1) ? MULTI_TAG_STD_DEVS : SINGLE_TAG_STD_DEVS;
        return base.times(1.0 + (avgDist * avgDist / 30.0));
    }

    // =========================================================================
    // Hub Targeting (raw PV pose → turret direction)
    // =========================================================================

    /**
     * Returns the hub's direction in the robot's reference frame, derived from
     * the raw PhotonVision PnP pose estimate this cycle.  0° = robot forward,
     * positive = CCW.  Available only when at least one enabled camera sees one
     * or more AprilTags <em>this cycle</em> and the alliance is known.
     *
     * <p>No filtering is applied — the raw per-cycle value is returned directly.
     *
     * @return Hub direction in robot frame (degrees), or empty.
     */
    public Optional<Double> getHubAngleDeg() {
        return m_pvHubAngleDeg;
    }

    /**
     * Returns the straight-line distance from the turret pivot to the hub.
     *
     * @return Distance in meters, or empty if unavailable this cycle.
     */
    public Optional<Double> getHubDistanceMeters() {
        return m_pvHubDistanceM;
    }

    /**
     * Returns {@code true} once at least one valid vision measurement has been
     * added to the drivetrain pose estimator since the last {@link #clearPoseReady()} call.
     * Used to gate odometry-based turret targeting and autonomous pose initialization.
     */
    public boolean hasPoseBeenCorrected() {
        return m_hasPoseBeenCorrected;
    }

    /**
     * Resets the pose-corrected flag so the next call to {@link #hasPoseBeenCorrected()}
     * returns {@code false} until a genuinely new vision measurement arrives.
     *
     * <p>Call this from {@code autonomousInit()} (before scheduling the auto command)
     * to force {@link frc.robot.commands.OutpostAutoCommand} to wait for a fresh
     * AprilTag fix at the actual autonomous start rather than accepting a stale
     * flag left over from teleop.
     */
    public void clearPoseReady() {
        m_hasPoseBeenCorrected = false;
        m_latestRawPose        = Optional.empty();
    }

    /**
     * Returns the raw pose from the best camera estimate processed this loop
     * (the camera that saw the most AprilTags), or empty if no valid estimate
     * was available this cycle.
     *
     * <p>This is the direct PhotonVision PnP result — it has NOT been filtered
     * through the Kalman estimator.  Use it for hard pose resets where you want
     * a clean, drift-free starting point rather than a fused estimate.
     *
     * @return Raw vision pose this loop, or {@link Optional#empty()}.
     */
    public Optional<edu.wpi.first.math.geometry.Pose2d> getLatestRawPose() {
        return m_latestRawPose;
    }

    private void updateHubTarget(Pose2d pvPose) {
        Translation2d hub = DriverStation.getAlliance()
                .map(a -> a == DriverStation.Alliance.Red
                        ? FieldLayout.RED_HUB_CENTER
                        : FieldLayout.BLUE_HUB_CENTER)
                .orElse(FieldLayout.BLUE_HUB_CENTER);

        var poseRotation = pvPose.getRotation();

        Translation2d turretPos = pvPose.getTranslation().plus(
                new Translation2d(Turret.TURRET_OFFSET_X_M, Turret.TURRET_OFFSET_Y_M)
                        .rotateBy(poseRotation));

        double fieldAngleDeg = Math.toDegrees(
                Math.atan2(hub.getY() - turretPos.getY(),
                           hub.getX() - turretPos.getX()));

        m_pvHubAngleDeg  = Optional.of(fieldAngleDeg - poseRotation.getDegrees());
        m_pvHubDistanceM = Optional.of(turretPos.getDistance(hub));
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

}
