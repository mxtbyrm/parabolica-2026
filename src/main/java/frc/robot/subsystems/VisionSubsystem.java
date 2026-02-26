package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Field;
import frc.robot.Constants.HubConstants;
import frc.robot.Constants.TrenchConstants;
import frc.robot.Constants.Vision;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

/**
 * Provides AprilTag targeting data for HUB aiming and TRENCH proximity detection,
 * and feeds global robot-pose measurements into the swerve drivetrain's pose
 * estimator via MegaTag2.
 *
 * <h2>Responsibilities</h2>
 * <ol>
 *   <li><b>HUB targeting</b> — selects the best visible HUB AprilTag each loop,
 *       exposes horizontal offset (tx), and computes distance via the height formula.</li>
 *   <li><b>TRENCH detection</b> — selects the nearest visible TRENCH tag and exposes
 *       distance to it; used by {@link frc.robot.subsystems.TrenchTraversalManager}
 *       for approach detection.</li>
 *   <li><b>Pose estimation</b> — every loop it calls
 *       {@link CommandSwerveDrivetrain#addVisionMeasurement} with the MegaTag2
 *       robot-pose estimate, filtered by rotation rate and tag count.  Standard
 *       deviations scale with distance and shrink with more visible tags.</li>
 * </ol>
 *
 * <h2>Distance Formula</h2>
 * <pre>
 *   distance = (tagHeight − CAMERA_HEIGHT_M) / tan(CAMERA_PITCH + ty)
 * </pre>
 * where {@code tagHeight} is selected from {@link LandmarkType}.
 *
 * <h2>Tag Selection Priority (HUB)</h2>
 * <p>Centered tags ({@link Field#HUB_CENTERED_TAG_IDS}) are preferred over offset tags.
 * Within each tier, the tag with the largest image area ({@code ta}) is selected.
 */
public class VisionSubsystem extends SubsystemBase {

    // =========================================================================
    // Landmark Type
    // =========================================================================

    /**
     * Identifies which field landmark the vision subsystem is currently targeting.
     * Switching the landmark changes the AprilTag height used in the distance formula
     * and which tags are selected each loop.
     */
    public enum LandmarkType {
        /**
         * HUB mode — targets HUB AprilTags for shooter aiming and scoring.
         * Uses {@link Field#ACTIVE_HUB_APRILTAG_HEIGHT_M} for distance calculation.
         */
        HUB,
        /**
         * TRENCH mode — targets TRENCH AprilTags for proximity / approach detection.
         * Uses {@link TrenchConstants#ACTIVE_TRENCH_APRILTAG_HEIGHT_M} for distance.
         */
        TRENCH
    }

    // =========================================================================
    // Dependencies
    // =========================================================================

    private final CommandSwerveDrivetrain m_drivetrain;

    // =========================================================================
    // Pose-estimator filter thresholds
    // =========================================================================

    /**
     * Maximum robot rotational speed (rotations/second) above which vision updates
     * are rejected.  High rotation introduces gyro-coupling artifacts in MegaTag2.
     */
    private static final double MAX_OMEGA_RPS_FOR_VISION = 2.0;

    /**
     * Minimum number of tags required for a pose estimate to be accepted.
     * Single-tag estimates are less reliable and have higher std devs.
     */
    private static final int MIN_TAG_COUNT = 1;

    // =========================================================================
    // Active Landmark
    // =========================================================================

    /** The landmark currently targeted.  Defaults to HUB. */
    private LandmarkType m_activeLandmark = LandmarkType.HUB;

    // =========================================================================
    // Cached results (updated each loop)
    // =========================================================================

    private Optional<Double>  m_distanceToHubM    = Optional.empty();
    private Optional<Double>  m_targetTxDeg        = Optional.empty();
    private OptionalInt       m_bestTagId          = OptionalInt.empty();
    private boolean           m_bestTagIsCentered  = false;

    private Optional<Double>  m_distanceToTrenchM  = Optional.empty();
    private OptionalInt       m_bestTrenchTagId     = OptionalInt.empty();
    /**
     * The raw fiducial for the best TRENCH tag; cached so {@code periodic()} can
     * access its per-fiducial {@code tync} angle for an accurate distance computation
     * even when the limelight's primary target ({@code getTY()}) is a HUB tag.
     */
    private RawFiducial       m_bestTrenchFiducial  = null;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Constructs the VisionSubsystem.
     *
     * @param drivetrain The swerve drivetrain whose pose estimator will receive
     *                   vision measurement updates each loop.
     */
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    // =========================================================================
    // Periodic — runs every robot loop
    // =========================================================================

    @Override
    public void periodic() {
        // 1. Provide the robot heading to MegaTag2 (required for yaw-resolved estimates).
        double headingDeg = m_drivetrain.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(Vision.LIMELIGHT_NAME, headingDeg, 0, 0, 0, 0, 0);

        // 2. Feed global pose into the drivetrain estimator.
        updatePoseEstimator();

        // 3. Fetch raw fiducials ONCE per loop to avoid duplicate network calls and
        //    ensure both selectors see a consistent snapshot.  Null-guard protects
        //    against Limelight disconnection / startup transients.
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Vision.LIMELIGHT_NAME);
        if (fiducials == null) {
            fiducials = new RawFiducial[0];
        }

        // 4. Select the best HUB tag and the best TRENCH tag for this loop cycle.
        //    TRENCH distance is computed inside selectBestTrenchTag() using the
        //    fiducial's own tync angle so it is accurate even when a HUB tag is
        //    the limelight's primary pipeline target.
        selectBestHubTag(fiducials);
        selectBestTrenchTag(fiducials);

        // 5. Compute HUB distance and tx from the chosen HUB tag.
        //    getTY() / getTX() are the limelight primary-target angles — correct
        //    when a HUB tag was selected as the pipeline crosshair target.
        if (m_bestTagId.isPresent()) {
            double ty = LimelightHelpers.getTY(Vision.LIMELIGHT_NAME);
            m_distanceToHubM = computeDistanceFromTy(ty, getTagHeightMeters(m_bestTagId.getAsInt()));
            m_targetTxDeg    = Optional.of(LimelightHelpers.getTX(Vision.LIMELIGHT_NAME));
        } else {
            m_distanceToHubM = Optional.empty();
            m_targetTxDeg    = Optional.empty();
        }
        // (TRENCH distance is computed in selectBestTrenchTag())

        // 6. Publish telemetry to SmartDashboard.
        publishTelemetry();
    }

    // =========================================================================
    // Public Accessors — HUB
    // =========================================================================

    /**
     * Returns the estimated horizontal distance from the robot to the HUB center
     * in meters.  Empty when no HUB tag is visible.
     *
     * @return Distance in meters, or {@link Optional#empty()}.
     */
    public Optional<Double> getDistanceToHubMeters() {
        return m_distanceToHubM;
    }

    /**
     * Returns the Limelight horizontal angle (tx) to the best HUB tag in degrees.
     * Positive = target is to the right of camera center.
     *
     * <p>To compute the turret correction: {@code delta = -tx} (negate because
     * camera-right requires a CCW turret rotation when the camera faces forward).
     *
     * @return Camera tx in degrees, or {@link Optional#empty()} if no tag visible.
     */
    public Optional<Double> getTargetTxDeg() {
        return m_targetTxDeg;
    }

    /**
     * Returns the ID of the best visible HUB AprilTag.
     *
     * @return Tag ID, or {@link OptionalInt#empty()} if no tag visible.
     */
    public OptionalInt getBestTagId() {
        return m_bestTagId;
    }

    /**
     * Returns {@code true} if the best visible tag is a face-centered tag (higher
     * confidence for pose estimation and aiming).
     */
    public boolean isBestTagCentered() {
        return m_bestTagIsCentered;
    }

    /**
     * Returns {@code true} if at least one valid HUB AprilTag is currently visible.
     */
    public boolean hasHubTarget() {
        return m_bestTagId.isPresent();
    }

    // =========================================================================
    // Public Accessors — TRENCH
    // =========================================================================

    /**
     * Returns the estimated distance to the nearest visible TRENCH AprilTag in meters.
     * Empty when no TRENCH tag is visible.
     *
     * @return Distance in meters, or {@link Optional#empty()}.
     */
    public Optional<Double> getDistanceToTrenchMeters() {
        return m_distanceToTrenchM;
    }

    /**
     * Returns {@code true} if at least one TRENCH AprilTag is currently visible.
     */
    public boolean hasTrenchTarget() {
        return m_bestTrenchTagId.isPresent();
    }

    // =========================================================================
    // Landmark Mode
    // =========================================================================

    /**
     * Sets the active landmark type.  This is informational — both HUB and TRENCH
     * tags are processed every loop regardless of this setting.  External consumers
     * may use this to know which dataset is "primary."
     *
     * @param landmark The landmark to set as active.
     */
    public void setActiveLandmark(LandmarkType landmark) {
        m_activeLandmark = landmark;
    }

    /** @return The currently active landmark. */
    public LandmarkType getActiveLandmark() {
        return m_activeLandmark;
    }

    // =========================================================================
    // Pose Estimator Integration
    // =========================================================================

    /**
     * Obtains a MegaTag2 robot-field pose estimate from the Limelight and, if it
     * passes quality filters, injects it into the drivetrain's Kalman filter.
     *
     * <h3>Quality filters</h3>
     * <ul>
     *   <li>At least {@link #MIN_TAG_COUNT} tags must be visible.</li>
     *   <li>Robot rotation rate must be ≤ {@link #MAX_OMEGA_RPS_FOR_VISION}.</li>
     * </ul>
     *
     * <h3>Standard deviation scaling</h3>
     * <pre>
     *   σ_xy = 0.4 + 0.3 × avgTagDist² / tagCount
     * </pre>
     */
    private void updatePoseEstimator() {
        boolean isRedAlliance = DriverStation.getAlliance()
                .map(a -> a == DriverStation.Alliance.Red)
                .orElse(false);

        PoseEstimate estimate = isRedAlliance
                ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(Vision.LIMELIGHT_NAME)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Vision.LIMELIGHT_NAME);

        if (estimate == null || estimate.tagCount < MIN_TAG_COUNT) {
            return;
        }

        double omegaRps = Math.abs(
                m_drivetrain.getState().Speeds.omegaRadiansPerSecond / (2.0 * Math.PI));
        if (omegaRps > MAX_OMEGA_RPS_FOR_VISION) {
            return;
        }

        double distSq   = estimate.avgTagDist * estimate.avgTagDist;
        double stddevXY = 0.4 + (0.3 * distSq / estimate.tagCount);

        // Heading stddev is set to MAX_VALUE intentionally: MegaTag2 uses the robot's
        // onboard gyro to lock the yaw, so the vision estimate's heading component
        // is already baked in.  Injecting a finite theta stddev would cause the Kalman
        // filter to fight the gyro, increasing heading drift.  MAX_VALUE tells the
        // estimator to trust only the XY component of this measurement.
        m_drivetrain.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(stddevXY, stddevXY, Double.MAX_VALUE));
    }

    // =========================================================================
    // Tag Selection — HUB
    // =========================================================================

    /**
     * Iterates the pre-fetched fiducial snapshot and picks the best HUB tag.
     * Centered tags take priority; within each tier the largest-area tag is preferred.
     *
     * @param fiducials Fiducial snapshot from this loop's {@code periodic()} call.
     */
    private void selectBestHubTag(RawFiducial[] fiducials) {
        RawFiducial bestCentered = null;
        RawFiducial bestOffset   = null;

        for (RawFiducial fid : fiducials) {
            if (!isHubTag(fid.id)) {
                continue;
            }
            // Reject high-ambiguity detections: when two tags produce similar pose
            // solutions Limelight reports ambiguity > 0 (0 = unique, 1 = degenerate).
            // The threshold of 0.15 is per Limelight documentation.
            if (fid.ambiguity > Vision.MAX_TAG_AMBIGUITY) {
                continue;
            }
            if (isCenteredTag(fid.id)) {
                if (bestCentered == null || fid.ta > bestCentered.ta) {
                    bestCentered = fid;
                }
            } else {
                if (bestOffset == null || fid.ta > bestOffset.ta) {
                    bestOffset = fid;
                }
            }
        }

        RawFiducial best = (bestCentered != null) ? bestCentered : bestOffset;
        if (best != null) {
            m_bestTagId         = OptionalInt.of(best.id);
            m_bestTagIsCentered = isCenteredTag(best.id);
        } else {
            m_bestTagId         = OptionalInt.empty();
            m_bestTagIsCentered = false;
        }
    }

    // =========================================================================
    // Tag Selection — TRENCH
    // =========================================================================

    /**
     * Iterates the pre-fetched fiducial snapshot and picks the largest-area TRENCH tag.
     * Distance is computed here using the fiducial's own {@code tync} angle so it
     * is accurate regardless of which tag the limelight's primary crosshair targets.
     *
     * @param fiducials Fiducial snapshot from this loop's {@code periodic()} call.
     */
    private void selectBestTrenchTag(RawFiducial[] fiducials) {
        RawFiducial best = null;

        for (RawFiducial fid : fiducials) {
            if (!isTrenchTag(fid.id)) {
                continue;
            }
            if (best == null || fid.ta > best.ta) {
                best = fid;
            }
        }

        m_bestTrenchFiducial = best;
        if (best != null) {
            m_bestTrenchTagId   = OptionalInt.of(best.id);
            // Use the per-fiducial vertical angle (tync, degrees) so the distance
            // formula is correct even when a HUB tag is the primary pipeline target.
            m_distanceToTrenchM = computeDistanceFromTy(
                    best.tync, TrenchConstants.ACTIVE_TRENCH_APRILTAG_HEIGHT_M);
        } else {
            m_bestTrenchTagId   = OptionalInt.empty();
            m_distanceToTrenchM = Optional.empty();
        }
    }

    // =========================================================================
    // Distance Computation
    // =========================================================================

    /**
     * Computes horizontal distance to a target using the vertical angle formula.
     *
     * @param ty            Limelight ty (vertical offset from crosshair) in degrees.
     * @param tagHeightM    Known height of the AprilTag center above carpet in meters.
     * @return Distance in meters, or empty if the angle is too near horizontal.
     */
    private Optional<Double> computeDistanceFromTy(double ty, double tagHeightM) {
        double angleRad = Units.degreesToRadians(Vision.CAMERA_PITCH_DEG)
                        + Units.degreesToRadians(ty);

        // Require a positive (upward-looking) combined angle so that
        //   distance = heightDelta / tan(angle) is always positive.
        // Near-zero angles → infinite distance; near-90° → division instability.
        if (angleRad > Math.toRadians(1.0) && angleRad < Math.toRadians(89.0)) {
            double heightDelta = tagHeightM - Vision.CAMERA_HEIGHT_M;
            double distance    = heightDelta / Math.tan(angleRad);
            if (distance > 0.0) {
                return Optional.of(distance);
            }
        }
        return Optional.empty();
    }

    // =========================================================================
    // Telemetry
    // =========================================================================

    private void publishTelemetry() {
        SmartDashboard.putString("Vision/ActiveLandmark", m_activeLandmark.name());
        SmartDashboard.putBoolean("Vision/HasHubTarget",    hasHubTarget());
        SmartDashboard.putBoolean("Vision/TagIsCentered",   m_bestTagIsCentered);
        SmartDashboard.putNumber("Vision/BestTagId",
                m_bestTagId.isPresent() ? m_bestTagId.getAsInt() : -1);
        SmartDashboard.putNumber("Vision/HubDistanceM",
                m_distanceToHubM.orElse(-1.0));
        SmartDashboard.putNumber("Vision/TargetTxDeg",
                m_targetTxDeg.orElse(0.0));
        SmartDashboard.putBoolean("Vision/HasTrenchTarget", hasTrenchTarget());
        SmartDashboard.putNumber("Vision/TrenchDistanceM",
                m_distanceToTrenchM.orElse(-1.0));
    }

    // =========================================================================
    // Static Membership Helpers
    // =========================================================================

    private static boolean isHubTag(int id) {
        for (int hubId : Field.HUB_ALL_TAG_IDS) {
            if (hubId == id) return true;
        }
        return false;
    }

    private static boolean isCenteredTag(int id) {
        for (int centerId : Field.HUB_CENTERED_TAG_IDS) {
            if (centerId == id) return true;
        }
        return false;
    }

    private static boolean isTrenchTag(int id) {
        for (int trenchId : TrenchConstants.TRENCH_ALL_TAG_IDS) {
            if (trenchId == id) return true;
        }
        return false;
    }

    // =========================================================================
    // Static Tag-Routing Helpers (public — used by ShooterKinematics, etc.)
    // =========================================================================

    /**
     * Returns the {@link LandmarkType} associated with a given AprilTag ID.
     *
     * <p>TRENCH tags (TE-26200/TE-26204) are IDs 1, 6, 7, 12 (Blue) and
     * 17, 22, 23, 28 (Red).  All other HUB tag IDs map to {@link LandmarkType#HUB}.
     *
     * @param tagId AprilTag ID.
     * @return {@link LandmarkType#TRENCH} for TRENCH tags; {@link LandmarkType#HUB} otherwise.
     */
    public static LandmarkType getLandmarkType(int tagId) {
        return switch (tagId) {
            case 1, 6, 7, 12, 17, 22, 23, 28 -> LandmarkType.TRENCH;
            default                            -> LandmarkType.HUB;
        };
    }

    /**
     * Returns the known AprilTag center height above carpet in meters for the given
     * tag ID, resolved to the current venue via {@link frc.robot.Constants.VisionConstants#IS_PRACTICE_FIELD}.
     *
     * @param tagId AprilTag ID.
     * @return Tag-center height in meters ({@link HubConstants#ACTIVE_HUB_APRILTAG_HEIGHT_M}
     *         or {@link TrenchConstants#ACTIVE_TRENCH_APRILTAG_HEIGHT_M}).
     */
    public static double getTagHeightMeters(int tagId) {
        return switch (getLandmarkType(tagId)) {
            case HUB    -> HubConstants.ACTIVE_HUB_APRILTAG_HEIGHT_M;
            case TRENCH -> TrenchConstants.ACTIVE_TRENCH_APRILTAG_HEIGHT_M;
        };
    }
}
