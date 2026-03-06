package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

    // ── Pose sanity-check thresholds ─────────────────────────────────────────
    // Reject vision poses that are clearly wrong before they corrupt the Kalman
    // filter.  These prevent the "teleport to field center" problem when PnP
    // returns a degenerate solution.

    /** Field boundary margin in meters.  Poses further than this outside the
     *  field walls are rejected outright (bad PnP solve). */
    private static final double FIELD_MARGIN_M = 1.0;

    /**
     * Maximum average tag distance (meters) for a single-tag estimate to be
     * trusted.  Beyond this, single-tag PnP is too noisy — std devs are set
     * to {@link Double#MAX_VALUE} so the Kalman filter effectively ignores
     * the measurement.  Multi-tag estimates are always accepted (their
     * redundancy removes the ambiguity problem).
     *
     * <p>Matches the YAGSL heuristic.
     */
    private static final double SINGLE_TAG_MAX_DIST_M = 4.0;

    // =========================================================================
    // State
    // =========================================================================

    private final CommandSwerveDrivetrain m_drivetrain;
    private final PhotonCamera[]          m_cameras;
    private final PhotonPoseEstimator[]   m_estimators;
    private final AprilTagFieldLayout     m_fieldLayout;
    private final Alert[]                 m_disconnectAlerts;

    // ── Hub targeting from raw PV pose (reset each cycle) ────────────────────

    /**
     * Robot-relative angle to the hub from the best PV estimate this cycle.
     * 0° = forward, positive = CCW.  Empty when no cameras see tags or alliance
     * is unknown.
     */
    private Optional<Double> m_pvHubAngleDeg = Optional.empty();

    /**
     * Straight-line distance from turret pivot to hub from the best PV estimate.
     */
    private Optional<Double> m_pvHubDistanceM = Optional.empty();

    /** Tag count of the best estimate this cycle (prefer multi-tag for heading accuracy). */
    private int m_bestEstimateTagCount = 0;

    /** True once at least one valid vision measurement has been added to the drivetrain. */
    private boolean m_hasPoseBeenCorrected = false;

    // ── EMA filter state for hub aiming (persists across cycles) ─────────────
    // Raw PnP heading jitters ±2-3° per cycle; this filter smooths the angle
    // and distance fed to the turret controller so isAligned() can stabilise.

    private double  m_filtHubAngleDeg = 0.0;
    private double  m_filtHubDistM    = 4.0;
    private boolean m_hubFilterSeeded = false;

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
            m_estimators[i] = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, ROBOT_TO_CAMERAS[i]);
            m_disconnectAlerts[i] = new Alert(
                    "PhotonVision camera '" + CAMERA_NAMES[i] + "' is disconnected.", AlertType.kWarning);
        }
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void periodic() {
        // Reject all vision updates when rotating too fast — motion blur and
        // timestamp latency errors make estimates unreliable at high angular velocity.
        // Reset per-cycle hub targeting state.
        m_pvHubAngleDeg = Optional.empty();
        m_pvHubDistanceM = Optional.empty();
        m_bestEstimateTagCount = 0;

        double rotRateDegPerS = Math.abs(Units.radiansToDegrees(
                m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()));
        boolean tooFast = rotRateDegPerS > PhotonVisionConstants.MAX_ROTATION_RATE_DEG_PER_S;

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

                if (tooFast) continue;

                // Only use coprocessor multi-tag PnP — the Pi handles all tag positions.
                // Single-tag fallback is omitted: it requires a local field layout and
                // produces out-of-bounds poses when the layout is empty.
                var optPose = m_estimators[i].estimateCoprocMultiTagPose(result);
                if (optPose.isEmpty()) continue;

                EstimatedRobotPose est = optPose.get();
                Pose2d visionPose2d = est.estimatedPose.toPose2d();
                int numTags = est.targetsUsed.size();

                // ── Field bounds check (YAGSL) ────────────────────────────
                double px = visionPose2d.getX();
                double py = visionPose2d.getY();
                if (px < -FIELD_MARGIN_M || px > FieldLayout.FIELD_LENGTH_M + FIELD_MARGIN_M
                        || py < -FIELD_MARGIN_M || py > FieldLayout.FIELD_WIDTH_M + FIELD_MARGIN_M) {
                    SmartDashboard.putBoolean("PhotonVision/" + CAMERA_LABELS[i] + "/RejectedOOB", true);
                    continue;
                }
                SmartDashboard.putBoolean("PhotonVision/" + CAMERA_LABELS[i] + "/RejectedOOB", false);

                // ── YAGSL-style std dev heuristic ─────────────────────────
                // No odometry comparison.  Trust is based purely on how
                // many tags are visible and how far away they are.
                //
                //  • Single tag >4 m  → MAX_VALUE std devs (effectively ignored)
                //  • Multi-tag        → lower base std devs
                //  • Distance scaling  → stdDevs × (1 + avgDist² / 30)
                //
                // This lets the Kalman filter converge naturally without
                // jump filters that can block valid corrections.
                double avgDist = averageTagDistanceM(est);

                double xyStdDev;
                double thetaStdDev;

                if (numTags == 1 && avgDist > SINGLE_TAG_MAX_DIST_M) {
                    // Single distant tag: too ambiguous to trust.
                    xyStdDev    = Double.MAX_VALUE;
                    thetaStdDev = Double.MAX_VALUE;
                } else {
                    double baseXY    = (numTags >= 2)
                            ? PhotonVisionConstants.MULTI_TAG_XY_STD_DEV_M
                            : PhotonVisionConstants.SINGLE_TAG_XY_STD_DEV_M;
                    double baseTheta = (numTags >= 2)
                            ? PhotonVisionConstants.MULTI_TAG_THETA_STD_DEV_RAD
                            : PhotonVisionConstants.SINGLE_TAG_THETA_STD_DEV_RAD;

                    double distFactor = 1.0 + (avgDist * avgDist / 30.0);
                    xyStdDev    = baseXY    * distFactor;
                    thetaStdDev = baseTheta * distFactor;
                }

                m_drivetrain.addVisionMeasurement(
                        visionPose2d,
                        est.timestampSeconds,
                        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
                m_hasPoseBeenCorrected = true;

                // Keep the estimate from the camera that saw the MOST tags
                // (multi-tag PnP has unambiguous heading → best turret aim).
                if (numTags >= m_bestEstimateTagCount) {
                    m_bestEstimateTagCount = numTags;
                    updateHubTarget(visionPose2d);
                }

                activeCameras++;

                SmartDashboard.putNumber("PhotonVision/" + CAMERA_LABELS[i] + "/NumTags", numTags);
                SmartDashboard.putNumber("PhotonVision/" + CAMERA_LABELS[i] + "/DistM",   avgDist);
                SmartDashboard.putNumber("PhotonVision/" + CAMERA_LABELS[i] + "/XYStdDev", xyStdDev);
            }
        }

        SmartDashboard.putNumber("PhotonVision/ActiveCameras",  activeCameras);
        SmartDashboard.putNumber("PhotonVision/Connected",      connectedCount);
        SmartDashboard.putBoolean("PhotonVision/TooFastSkip",   tooFast);
        SmartDashboard.putNumber("PhotonVision/HubAngleDeg",    m_pvHubAngleDeg.orElse(0.0));
        SmartDashboard.putNumber("PhotonVision/HubDistanceM",   m_pvHubDistanceM.orElse(-1.0));
        SmartDashboard.putBoolean("PhotonVision/HasHubTarget",  m_pvHubAngleDeg.isPresent());
    }

    // =========================================================================
    // Hub Targeting (raw PV pose → turret direction)
    // =========================================================================

    /**
     * Returns the hub's direction in the robot's reference frame, derived from
     * the EMA-filtered raw PhotonVision PnP pose estimate.  0° = robot forward,
     * positive = CCW.  Available only when at least one enabled camera sees one
     * or more AprilTags <em>this cycle</em> and the alliance is known.
     *
     * <p>The raw PnP heading jitters ±2-3° per cycle (single-tag).  An EMA
     * filter with α = {@link PhotonVisionConstants#PV_HUB_AIM_ALPHA} smooths
     * this jitter so the turret can settle within its 1° alignment tolerance
     * and {@code isReadyToShoot()} becomes {@code true}.
     *
     * <p>Unlike {@link VisionSubsystem#getHubRobotRelativeAngleDeg()} (which uses
     * the fused pose estimator), this method uses the <b>raw PnP-solved pose</b>
     * from the camera pipeline.  The raw pose has <em>unblended</em> heading,
     * which is more accurate for turret aim when the fused estimator's theta
     * trust is conservatively low.
     *
     * <p>Priority chain for turret targeting:
     * <ol>
     *   <li>Odometry hub angle (fused estimator — correctly accounts for turret
     *       pivot offset; always available; primary)</li>
     *   <li><b>PhotonVision hub angle</b> (this method — EMA-filtered, gyro-corrected
     *       PnP heading; secondary)</li>
     *   <li>Limelight tx (direct camera feedback — last-resort fallback)</li>
     * </ol>
     *
     * @return Filtered hub direction in robot frame (degrees), or empty.
     */
    public Optional<Double> getHubAngleDeg() {
        return m_pvHubAngleDeg;
    }

    /**
     * Returns the straight-line distance from the turret pivot to the hub,
     * derived from the EMA-filtered raw PhotonVision pose estimate.
     *
     * @return Filtered distance in meters, or empty if unavailable this cycle.
     */
    public Optional<Double> getHubDistanceMeters() {
        return m_pvHubDistanceM;
    }

    /**
     * Returns {@code true} once at least one valid vision measurement has been
     * added to the drivetrain pose estimator.  Used by the Superstructure to
     * gate odometry-based turret targeting — the fused pose is only trustworthy
     * after vision has corrected it at least once.
     */
    public boolean hasPoseBeenCorrected() {
        return m_hasPoseBeenCorrected;
    }

    /**
     * Computes the turret-to-hub angle and distance from a raw PV field pose,
     * then applies an EMA filter to suppress cycle-to-cycle heading jitter.
     *
     * <p>Raw single-tag PnP heading jitters ±2-3°.  The turret alignment
     * tolerance is 1°.  Without filtering, the turret chases a noisy setpoint
     * each cycle and {@code isAligned()} never stabilises — the robot gets
     * stuck in PREPPING_TO_SHOOT.  The EMA with
     * α = {@link PhotonVisionConstants#PV_HUB_AIM_ALPHA} reduces jitter to
     * roughly ±0.6° while settling within ~220 ms of a step change.
     *
     * <p>Filter state ({@code m_filtHubAngleDeg}, {@code m_filtHubDistM})
     * persists across cycles where cameras temporarily lose tags, so the
     * filter resumes tracking smoothly when tags reappear rather than
     * re-seeding from scratch.
     */
    private void updateHubTarget(Pose2d pvPose) {
        // Default to Blue hub when DS has not yet reported an alliance (bench / practice).
        Translation2d hub = DriverStation.getAlliance()
                .map(a -> a == DriverStation.Alliance.Red
                        ? FieldLayout.RED_HUB_CENTER
                        : FieldLayout.BLUE_HUB_CENTER)
                .orElse(FieldLayout.BLUE_HUB_CENTER);

        // Use the gyro-fused heading instead of raw PnP heading for the
        // turret-offset rotation and robot-relative conversion.  Raw PnP
        // heading from a single tag jitters ±2-3° and has systematic skew;
        // the Pigeon 2 gyro is far more stable and accurate for heading.
        // PV translation (x, y on the field) is still used — only the
        // rotation component is replaced.
        var gyroRotation = m_drivetrain.getState().Pose.getRotation();

        Translation2d turretPos = pvPose.getTranslation().plus(
                new Translation2d(Turret.TURRET_OFFSET_X_M, Turret.TURRET_OFFSET_Y_M)
                        .rotateBy(gyroRotation));

        double fieldAngleDeg = Math.toDegrees(
                Math.atan2(hub.getY() - turretPos.getY(),
                           hub.getX() - turretPos.getX()));
        double rawAngleDeg = fieldAngleDeg - gyroRotation.getDegrees();
        double rawDistM    = turretPos.getDistance(hub);

        // EMA filter: smooths raw PnP jitter while tracking the true aim.
        double alpha = PhotonVisionConstants.PV_HUB_AIM_ALPHA;
        if (!m_hubFilterSeeded) {
            m_filtHubAngleDeg = rawAngleDeg;
            m_filtHubDistM    = rawDistM;
            m_hubFilterSeeded = true;
        } else {
            m_filtHubAngleDeg += alpha * (rawAngleDeg - m_filtHubAngleDeg);
            m_filtHubDistM    += alpha * (rawDistM    - m_filtHubDistM);
        }

        m_pvHubAngleDeg  = Optional.of(m_filtHubAngleDeg);
        m_pvHubDistanceM = Optional.of(m_filtHubDistM);
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
