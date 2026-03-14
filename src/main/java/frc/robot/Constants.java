package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.util.Units;

/**
 * Central repository for all robot-wide constants.
 *
 * <p>No magic numbers appear elsewhere in the codebase. All internal computations
 * use SI units (meters, radians, seconds); imperial conversions are performed here
 * via {@link Units} solely for human-readable declarations.
 *
 * <p>Nested classes group constants by mechanism or domain.
 *
 * <h2>AndyMark Field vs. Official Field</h2>
 * <p>Set {@link VisionConstants#IS_ANDYMARK_FIELD} to {@code true} at a regional using an
 * AndyMark field (TE-26300).  That single flag propagates to both HUB and TRENCH AprilTag
 * heights automatically.
 */
public final class Constants {

    private Constants() {} // Utility class — do not instantiate.

    // =========================================================================
    // VisionConstants — AndyMark / Official Field Toggle
    // =========================================================================

    /**
     * Vision-related field-configuration constants.
     *
     * <p><b>IS_ANDYMARK_FIELD is the single source of truth for venue selection.</b>
     * It controls AprilTag height resolution for both HUB and TRENCH:
     * <ul>
     *   <li>HUB tags:    Official = 44.25 in | AndyMark (TE-26300) = 45.0625 in (+0.8125 in)</li>
     *   <li>TRENCH tags: Official = 35.0 in  | AndyMark (TE-26300) = 35.6875 in (+0.6875 in)</li>
     * </ul>
     */
    public static final class VisionConstants {

        /**
         * Master vision enable flag.  Set {@code false} when the Limelight is not
         * physically installed or configured.  When {@code false}, VisionSubsystem
         * skips all Limelight network calls and returns empty optionals from every
         * accessor, so all callers fall back to odometry-only operation.
         * Flip to {@code true} once the camera is wired, focused, and its pipeline
         * IDs are confirmed.
         */
        public static final boolean VISION_ENABLED = false;

        /**
         * Set {@code true} when the field is an AndyMark field (TE-26300) — used at
         * some regionals as well as practice venues.
         *
         * <p>This flag <em>only</em> affects AprilTag mounting heights:
         * {@link Field#ACTIVE_HUB_APRILTAG_HEIGHT_M} and
         * {@link TrenchConstants#ACTIVE_TRENCH_APRILTAG_HEIGHT_M}.
         * It does NOT affect the HUB shift schedule; use {@link #IS_PRACTICE_MODE} for that.
         */
        public static final boolean IS_ANDYMARK_FIELD = false;

        /**
         * Set {@code true} during practice sessions (no FMS shift schedule).
         * When {@code true}, {@link frc.robot.util.HubStateMonitor} always reports
         * {@code HubState.ACTIVE} regardless of match time, so the HUB shift schedule
         * is bypassed entirely.
         *
         * <p>This flag is independent of {@link #IS_ANDYMARK_FIELD}: you can practice
         * on either an AndyMark or a welded field, and you can compete at a real regional
         * on an AndyMark field with this flag set to {@code false}.
         */
        public static final boolean IS_PRACTICE_MODE = false;
    }

    // =========================================================================
    // Field / Game Constants
    // =========================================================================

    /**
     * Game-piece (FUEL) and HUB geometry constants.
     * Tag ID arrays here are the primary source for tag membership checks;
     * see {@link HubConstants} for per-side layout details.
     */
    public static final class Field {

        // --- FUEL (Game Piece) -----------------------------------------------

        /** Diameter of the FUEL game piece in meters (5.91 in). */
        public static final double FUEL_DIAMETER_M = Units.inchesToMeters(5.91);  // 0.15011 m

        /** Radius of the FUEL game piece in meters. */
        public static final double FUEL_RADIUS_M = FUEL_DIAMETER_M / 2.0;         // 0.07506 m

        /**
         * Midpoint mass of the FUEL game piece in kilograms.
         * Spec range 0.203–0.227 kg; midpoint = 0.215 kg.
         * Source: 2026 REBUILT Game Manual.
         */
        public static final double FUEL_MASS_KG = 0.215;

        // --- HUB Geometry — Official Competition Field -----------------------

        /**
         * Height of the HUB AprilTag centers above carpet on the official field.
         * Source: 2026 REBUILT Game Manual / TE-26300 v2, Page 9–10.
         */
        public static final double HUB_APRILTAG_HEIGHT_OFFICIAL_M =
                Units.inchesToMeters(44.25);  // 1.1240 m

        /** Height of the HUB top opening front edge above carpet (scoring rim). */
        public static final double HUB_TOP_OPENING_HEIGHT_M =
                Units.inchesToMeters(72.0);   // 1.8288 m

        /** Height of the HUB geometric center above carpet (58.125 in = 1.4764 m).
         *  This is the aim point where the ball should arrive after clearing the rim. */
        public static final double HUB_CENTER_HEIGHT_M =
                Units.inchesToMeters(58.125); // 1.4764 m

        /** Width of the hexagonal HUB top opening across flats. */
        public static final double HUB_TOP_OPENING_DIAMETER_M =
                Units.inchesToMeters(41.7);   // 1.0592 m

        /** Side length of the square HUB base footprint (47.0 in). */
        public static final double HUB_BASE_WIDTH_M =
                Units.inchesToMeters(47.0);   // 1.1938 m

        /** Distance from the alliance wall to the HUB front face/wall (158.6 in).
         *  Hub geometric center = this + HUB_BASE_WIDTH_M / 2.0 = 182.1 in. */
        public static final double HUB_DIST_FROM_ALLIANCE_WALL_M =
                Units.inchesToMeters(156.06);  // 3.9651 m

        // --- HUB Geometry — AndyMark Field (TE-26300) ------------------------

        /**
         * Height of the HUB AprilTag centers above carpet on the AndyMark field (TE-26300).
         *
         * <p>Derivation from TE-26300 v2 AprilTag Placement diagrams (Pages 9–10):
         * <ul>
         *   <li>Bottom edge of AprilTag black square = <b>41.0 in</b> above carpet (all 4 sides).</li>
         *   <li>AprilTag square size = 8.125 in.</li>
         *   <li>Tag center = 41.0 + 8.125/2 = <b>45.0625 in = 1.1446 m</b>.</li>
         * </ul>
         * Top of TE-26306 plate = 49.5 in (Assembly Step 6).
         * Delta vs. official: <b>+0.8125 in (+20.6 mm)</b>.
         */
        public static final double HUB_APRILTAG_HEIGHT_PRACTICE_M =
                Units.inchesToMeters(45.0625);  // 1.1446 m

        /** Alias for {@link VisionConstants#IS_ANDYMARK_FIELD}. */
        public static final boolean IS_ANDYMARK_FIELD = VisionConstants.IS_ANDYMARK_FIELD;

        /**
         * Resolved AprilTag height for the current venue, selected by
         * {@link VisionConstants#IS_ANDYMARK_FIELD}.  All vision distance calculations
         * should reference this constant (or use
         * {@link frc.robot.subsystems.VisionSubsystem#getTagHeightMeters(int)} for
         * per-tag routing).
         */
        public static final double ACTIVE_HUB_APRILTAG_HEIGHT_M =
                VisionConstants.IS_ANDYMARK_FIELD
                        ? HUB_APRILTAG_HEIGHT_PRACTICE_M
                        : HUB_APRILTAG_HEIGHT_OFFICIAL_M;

        // --- AprilTag Physical Specifications --------------------------------

        /** Side length of each HUB AprilTag square in meters (8.125 in). */
        public static final double APRILTAG_SQUARE_SIZE_M =
                Units.inchesToMeters(8.125);  // 0.20638 m

        /** AprilTag family used on the HUB. */
        public static final String APRILTAG_FAMILY = "36h11";

        // --- HUB AprilTag IDs by Face ----------------------------------------
        //
        // Source: TE-26300 v2 AprilTag Placement diagrams (Pages 9–10).
        //
        // Each face has two tags: one at the CENTERED horizontal position
        // (19.78 in from left edge) and one at the OFFSET position (5.78 in
        // from the near edge).  On Sides 1–3 the offset tag is to the LEFT;
        // on Side 4 the offset tag is to the RIGHT (mirrored).
        //
        // Assembly note: "All AprilTags are positioned in the center and the
        // left side except on Side 4 where it is positioned towards the center
        // and right side."
        //
        // FORMAT: [Blue Alliance ID, Red Alliance ID]
        //
        // Side 1 ──────────────────────────────────────────────────────────────
        /** Blue HUB — Side 1 centered (19.78 in from left edge, right tag). */
        public static final int BLUE_HUB_SIDE1_CENTER = 4;
        /** Blue HUB — Side 1 offset-left (5.78 in from left edge, left tag). */
        public static final int BLUE_HUB_SIDE1_OFFSET = 3;

        /** Red HUB — Side 1 centered. */
        public static final int RED_HUB_SIDE1_CENTER = 20;
        /** Red HUB — Side 1 offset-left. */
        public static final int RED_HUB_SIDE1_OFFSET = 19;

        // Side 2 ──────────────────────────────────────────────────────────────
        /** Blue HUB — Side 2 centered (right tag). */
        public static final int BLUE_HUB_SIDE2_CENTER = 2;
        /** Blue HUB — Side 2 offset-left (left tag). */
        public static final int BLUE_HUB_SIDE2_OFFSET = 11;

        /** Red HUB — Side 2 centered. */
        public static final int RED_HUB_SIDE2_CENTER = 18;
        /** Red HUB — Side 2 offset-left. */
        public static final int RED_HUB_SIDE2_OFFSET = 27;

        // Side 3 ──────────────────────────────────────────────────────────────
        /** Blue HUB — Side 3 centered (right tag). */
        public static final int BLUE_HUB_SIDE3_CENTER = 10;
        /** Blue HUB — Side 3 offset-left (left tag). */
        public static final int BLUE_HUB_SIDE3_OFFSET = 9;

        /** Red HUB — Side 3 centered. */
        public static final int RED_HUB_SIDE3_CENTER = 26;
        /** Red HUB — Side 3 offset-left. */
        public static final int RED_HUB_SIDE3_OFFSET = 25;

        // Side 4 — MIRRORED ───────────────────────────────────────────────────
        /**
         * Blue HUB — Side 4 centered (19.78 in from left edge = LEFT tag for Side 4).
         * Side 4 is the mirror of Sides 1–3: the centered tag is at the LEFT position
         * and the offset tag is at the RIGHT (5.78 in from the right edge).
         */
        public static final int BLUE_HUB_SIDE4_CENTER = 5;
        /**
         * Blue HUB — Side 4 offset-RIGHT (5.78 in from right edge = 40.283 in from left).
         * Unlike Sides 1–3, the offset tag is on the RIGHT on this face.
         */
        public static final int BLUE_HUB_SIDE4_OFFSET = 8;

        /** Red HUB — Side 4 centered (left tag). */
        public static final int RED_HUB_SIDE4_CENTER = 21;
        /** Red HUB — Side 4 offset-right. */
        public static final int RED_HUB_SIDE4_OFFSET = 24;

        /**
         * All 16 HUB AprilTag IDs.
         * Source: 2026 REBUILT Game Manual (IDs 2–5, 8–11, 18–21, 24–27).
         * Use for membership checks rather than hard-coded ID comparisons.
         */
        public static final int[] HUB_ALL_TAG_IDS = {
            BLUE_HUB_SIDE1_CENTER, BLUE_HUB_SIDE1_OFFSET,
            BLUE_HUB_SIDE2_CENTER, BLUE_HUB_SIDE2_OFFSET,
            BLUE_HUB_SIDE3_CENTER, BLUE_HUB_SIDE3_OFFSET,
            BLUE_HUB_SIDE4_CENTER, BLUE_HUB_SIDE4_OFFSET,
            RED_HUB_SIDE1_CENTER,  RED_HUB_SIDE1_OFFSET,
            RED_HUB_SIDE2_CENTER,  RED_HUB_SIDE2_OFFSET,
            RED_HUB_SIDE3_CENTER,  RED_HUB_SIDE3_OFFSET,
            RED_HUB_SIDE4_CENTER,  RED_HUB_SIDE4_OFFSET,
        };
        // Sanity: {4,3, 2,11, 10,9, 5,8, 20,19, 18,27, 26,25, 21,24}

        /**
         * IDs of the "centered" tag on each HUB face (19.78 in from the left edge on
         * Sides 1–3; left-position on Side 4).  These tags are preferred by
         * {@link frc.robot.subsystems.VisionSubsystem} for turret aiming because
         * they are closer to the face centerline and have higher pose-estimation
         * confidence.
         */
        public static final int[] HUB_CENTERED_TAG_IDS = {
            BLUE_HUB_SIDE1_CENTER, BLUE_HUB_SIDE2_CENTER,
            BLUE_HUB_SIDE3_CENTER, BLUE_HUB_SIDE4_CENTER,
            RED_HUB_SIDE1_CENTER,  RED_HUB_SIDE2_CENTER,
            RED_HUB_SIDE3_CENTER,  RED_HUB_SIDE4_CENTER,
        };
        // = {4, 2, 10, 5, 20, 18, 26, 21}
    }

    // =========================================================================
    // HubConstants — Comprehensive HUB Specification (TE-26300 v2, 2026-01-12)
    // =========================================================================

    /**
     * Detailed HUB physical and AprilTag specifications sourced directly from the
     * TE-26300 v2 engineering drawings (updated 2026-01-12).
     *
     * <p>For tag membership checks and vision logic, prefer {@link Field#HUB_ALL_TAG_IDS}
     * and {@link Field#HUB_CENTERED_TAG_IDS}.  This class provides the raw structural
     * dimensions for simulation, CAD verification, and mechanical clearance checks.
     */
    public static final class HubConstants {

        // ── Official Field Physical Dimensions ────────────────────────────────

        /** Official HUB base footprint (square, 47.0 in). */
        public static final double HUB_BASE_WIDTH_M =
                Units.inchesToMeters(47.0);  // 1.1938 m

        /** Distance from the alliance wall to the HUB front face/wall (158.6 in).
         *  Hub geometric center = this + HUB_BASE_WIDTH_M / 2.0 = 182.1 in. */
        public static final double HUB_DIST_FROM_ALLIANCE_WALL_M =
                Units.inchesToMeters(156.06); // 4.0284 m

        /** Width of the hexagonal top opening across flats (41.7 in). */
        public static final double HUB_TOP_OPENING_WIDTH_M =
                Units.inchesToMeters(41.7);  // 1.0592 m

        /** Height of the front rim of the top opening above carpet (72.0 in). */
        public static final double HUB_TOP_OPENING_HEIGHT_M =
                Units.inchesToMeters(72.0);  // 1.8288 m

        /** Height of the HUB geometric center above carpet (58.125 in = 1.4764 m).
         *  This is the aim point where the ball should arrive after clearing the rim. */
        public static final double HUB_CENTER_HEIGHT_M =
                Units.inchesToMeters(58.125); // 1.4764 m

        // ── AprilTag Vision Heights ───────────────────────────────────────────

        /**
         * Center height of HUB AprilTags on the OFFICIAL competition field (44.25 in).
         * Source: 2026 REBUILT Game Manual.
         */
        public static final double HUB_APRILTAG_HEIGHT_OFFICIAL_M =
                Units.inchesToMeters(44.25); // 1.1240 m

        /**
         * Center height of HUB AprilTags on the TE-26300 practice HUB (45.0625 in).
         * Source: TE-26300 v2 AprilTag Placement diagrams, Pages 9–10.
         * <ul>
         *   <li>Bottom edge of black square = 41.0 in above carpet (all 4 sides).</li>
         *   <li>Tag size = 8.125 in → center = 41.0 + 8.125/2 = 45.0625 in.</li>
         * </ul>
         * Top of TE-26306 plate = 49.5 in (Assembly Step 6).
         */
        public static final double HUB_APRILTAG_HEIGHT_PRACTICE_M =
                Units.inchesToMeters(45.0625); // 1.1446 m

        /**
         * Height delta: practice tags are +0.8125 in (20.6 mm) higher than official.
         * Uncompensated, this error directly offsets vision pose estimation.
         */
        public static final double HUB_APRILTAG_HEIGHT_DELTA_M =
                HUB_APRILTAG_HEIGHT_PRACTICE_M - HUB_APRILTAG_HEIGHT_OFFICIAL_M; // +0.02064 m

        /**
         * Resolved HUB AprilTag height for the current venue, controlled by
         * {@link VisionConstants#IS_ANDYMARK_FIELD}.
         */
        public static final double ACTIVE_HUB_APRILTAG_HEIGHT_M =
                VisionConstants.IS_ANDYMARK_FIELD
                        ? HUB_APRILTAG_HEIGHT_PRACTICE_M
                        : HUB_APRILTAG_HEIGHT_OFFICIAL_M;

        /** AprilTag square size (36h11 family, 8.125 in). */
        public static final double APRILTAG_SIZE_M =
                Units.inchesToMeters(8.125); // 0.20638 m

        // ── AprilTag IDs by Side ──────────────────────────────────────────────
        // Each array is [Blue Alliance ID, Red Alliance ID].
        // "Left" and "Right" refer to position as viewed from outside the HUB.

        /** Side 1: left/offset position — [Blue=3, Red=19]. Offset: 5.78 in from left edge. */
        public static final int[] HUB_SIDE1_LEFT_IDS  = {3, 19};
        /** Side 1: right/centered position — [Blue=4, Red=20]. Offset: 19.78 in from left edge. */
        public static final int[] HUB_SIDE1_RIGHT_IDS = {4, 20};

        /** Side 2: left/offset — [Blue=11, Red=27]. */
        public static final int[] HUB_SIDE2_LEFT_IDS  = {11, 27};
        /** Side 2: right/centered — [Blue=2, Red=18]. */
        public static final int[] HUB_SIDE2_RIGHT_IDS = {2, 18};

        /** Side 3: left/offset — [Blue=9, Red=25]. */
        public static final int[] HUB_SIDE3_LEFT_IDS  = {9, 25};
        /** Side 3: right/centered — [Blue=10, Red=26]. */
        public static final int[] HUB_SIDE3_RIGHT_IDS = {10, 26};

        /**
         * Side 4: left/centered — [Blue=5, Red=21]. Offset: 19.78 in from left edge.
         * Side 4 is the <b>mirror</b> of Sides 1–3: the centered tag is at the LEFT
         * position; the offset tag is at the RIGHT (5.78 in from the right edge).
         */
        public static final int[] HUB_SIDE4_LEFT_IDS  = {5, 21};   // centered position
        /**
         * Side 4: right/offset — [Blue=8, Red=24].
         * Offset: 5.78 in from right edge = 46.063 − 5.78 = 40.283 in from left edge.
         */
        public static final int[] HUB_SIDE4_RIGHT_IDS = {8, 24};   // offset position

        /** All 16 HUB AprilTag IDs (IDs 2–5, 8–11, 18–21, 24–27). */
        public static final int[] HUB_ALL_APRILTAG_IDS =
            {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};

        // ── Horizontal AprilTag Offsets on TE-26306 Plate ─────────────────────
        // Plate width = 46.063 in. Measured from the left edge of the face plate.

        /** Centered tag horizontal position from left edge (Sides 1–3 right tag, Side 4 left tag). */
        public static final double HUB_TAG_CENTER_OFFSET_M =
                Units.inchesToMeters(19.78);

        /** Offset tag horizontal position from left edge (Sides 1–3 left tag). */
        public static final double HUB_TAG_LEFT_OFFSET_M =
                Units.inchesToMeters(5.78);

        /**
         * Side 4 offset tag: position from left edge = 46.063 − 5.78 = 40.283 in.
         * Equivalently, 5.78 in from the right edge.
         */
        public static final double HUB_TAG_SIDE4_RIGHT_OFFSET_M =
                Units.inchesToMeters(46.063 - 5.78); // 40.283 in from left

        // ── Practice HUB Structural Dimensions (TE-26300 v2) ─────────────────

        /** TE-26301: Hub Main Vertical Beam height (4× used). */
        public static final double TE26301_HEIGHT_M         = Units.inchesToMeters(55.972);

        /** TE-26302: Hub Top Assembly overall size (square). */
        public static final double TE26302_OVERALL_M        = Units.inchesToMeters(46.063);
        /** TE-26302: Hexagonal opening across flats. */
        public static final double TE26302_HEX_OPENING_M    = Units.inchesToMeters(41.727);
        /** TE-26302: Funnel assembly height above the top plate. */
        public static final double TE26302_FUNNEL_HEIGHT_M  = Units.inchesToMeters(19.528);

        /** TE-26303: Hub Main Horizontal Beam length (6× used). */
        public static final double TE26303_LENGTH_M         = Units.inchesToMeters(43.063);

        /** TE-26304: Hub Top Funnel Plate width (23.031 in, 2× used). */
        public static final double TE26304_WIDTH_M          = Units.inchesToMeters(23.031);
        /** TE-26304: Hub Top Funnel Plate length (46.063 in). */
        public static final double TE26304_LENGTH_M         = Units.inchesToMeters(46.063);

        /** TE-26305: Hub Bottom Support Plate width (10.500 in, 4× used). */
        public static final double TE26305_WIDTH_M          = Units.inchesToMeters(10.500);
        /** TE-26305: Hub Bottom Support Plate length (46.063 in). */
        public static final double TE26305_LENGTH_M         = Units.inchesToMeters(46.063);

        /** TE-26306: Hub AprilTag Plate width (10.500 in, 4× used, one per side). */
        public static final double TE26306_WIDTH_M          = Units.inchesToMeters(10.500);
        /** TE-26306: Hub AprilTag Plate length (46.063 in). */
        public static final double TE26306_LENGTH_M         = Units.inchesToMeters(46.063);
        /**
         * Top edge of the TE-26306 plate above carpet (49.5 in).
         * Source: Assembly Step 6, TE-26300 v2.
         * Confirms tag bottom edge = 49.5 − 10.5 = 39.0 in; however the placement
         * diagram explicitly states the black-square bottom = 41.0 in, which accounts
         * for the inset of the tag within the plate opening.
         */
        public static final double TE26306_TOP_EDGE_HEIGHT_M =
                Units.inchesToMeters(49.5);

        /** GE-26329: Hub Funnel Side panel flat height (17.90 in, 6× used). */
        public static final double GE26329_HEIGHT_M         = Units.inchesToMeters(17.90);
        /** GE-26329: Hub Funnel Side panel flat width (24.09 in). */
        public static final double GE26329_WIDTH_M          = Units.inchesToMeters(24.09);
        /**
         * GE-26329: Panel angle from horizontal (73.9°).
         * Assembled with 50-lb cable ties through 12× Ø0.25 in holes.
         */
        public static final double GE26329_ANGLE_DEG        = 73.9;
        public static final double GE26329_ANGLE_RAD        = Math.toRadians(73.9);
    }

    // =========================================================================
    // Shooter Constants (Flywheel + Hood)
    // =========================================================================

    /** Constants for the flywheel and hood mechanisms. */
    public static final class Shooter {

        // CAN IDs (on "CANivore" bus)
        public static final int FLYWHEEL_CAN_ID = 21;
        public static final int HOOD_CAN_ID     = 20;

        // --- CANrange sensor (CAN bus) ----------------------------------------

        /**
         * Master enable flag for the shooter CANrange proximity sensor.
         *
         * <p>Set {@code false} when the CANrange has not yet been wired or configured.
         * When {@code false}, {@link frc.robot.RobotContainer} skips registering the
         * shot-counter trigger entirely — the sensor object is still created so the
         * code compiles, but no distance reads occur and the ball count is never
         * automatically decremented.
         *
         * <p>Flip to {@code true} once the sensor is physically connected, assigned
         * the CAN ID below, and confirmed to report sensible distance values.
         */
        public static final boolean SHOOTER_CANRANGE_ENABLED = false;

        /**
         * CAN ID of the CANrange proximity sensor mounted between the feeder exit
         * and the flywheel contact zone.  When a ball is present the measured
         * distance drops below {@link #SHOOTER_CANRANGE_THRESHOLD_M}; the rising
         * edge (ball clears the sensor) confirms the shot and decrements the ball count.
         *
         * <p>Update this ID to match the physical CAN configuration.
         */
        public static final int    SHOOTER_CANRANGE_CAN_ID      = 30; // NOTE: must differ from HOOD_CAN_ID (20)

        /**
         * Distance threshold in metres below which the CANrange considers a ball
         * present.  Tune on the real robot: hold a ball in front of the sensor and
         * pick a value comfortably below the no-ball reading.
         */
        public static final double SHOOTER_CANRANGE_THRESHOLD_M = 0.10;

        // --- Gear Ratios (motor rotations : mechanism rotations) -------------

        /** Flywheel gear ratio: 1.5 : 1 from motor shaft to flywheel contact wheel. */
        public static final double FLYWHEEL_GEAR_RATIO = 1.5;

        /** Hood gear ratio: 21 : 1 from motor shaft to hood pivot. */
        public static final double HOOD_GEAR_RATIO = 52.5;

        /**
         * Hood motor inversion.  Motor CCW → belt CCW → gear CCW → hood CW → hood lifts.
         * {@code CounterClockwise_Positive} (Phoenix 6 default) means CCW motor rotation
         * produces a positive encoder reading, so positive position commands increase the
         * hood angle (raise the launch angle) as expected.
         */
        public static final InvertedValue HOOD_INVERT = InvertedValue.CounterClockwise_Positive;

        /**
         * Flywheel motor inversion. Set to {@code Clockwise_Positive} if the motor must
         * spin clockwise (when viewed from the shaft end) to produce forward ball launch.
         * Flip to {@code CounterClockwise_Positive} if balls launch in reverse or the
         * wheel spins the wrong direction on a positive RPM command.
         */
        public static final InvertedValue FLYWHEEL_INVERT = InvertedValue.Clockwise_Positive;

        // --- Flywheel Physical -----------------------------------------------

        /**
         * Shooter wheel assembly overview:
         *
         * <ul>
         *   <li><b>Bottom (main flywheel):</b> 3 × 4-inch-diameter plastic compliance
         *       wheels + 2 × 4-inch-diameter 0.7 kg steel inertia wheels, all on the
         *       same shaft.  The steel wheels provide rotational inertia to reduce RPM
         *       droop between consecutive shots (faster recovery time).</li>
         *   <li><b>Top (hood wheel rows):</b> 2 × rows of 2-inch-diameter plastic
         *       compliance wheels.  These counter-rotate at twice the bottom-wheel RPM
         *       (achieved through the fixed mechanical gearing) so that top and bottom
         *       contact surfaces move at <em>identical tangential speeds</em>.  Equal
         *       surface speeds from opposite sides impart zero net spin on the ball,
         *       eliminating Magnus-effect trajectory deviation.</li>
         *   <li><b>Drive:</b> a single motor (CAN ID {@link #FLYWHEEL_CAN_ID}) drives
         *       both wheel assemblies through fixed gearing.  {@link #FLYWHEEL_GEAR_RATIO}
         *       is the motor-to-<em>bottom-wheel</em> reduction.</li>
         * </ul>
         *
         * <p>Ball launch speed is derived from the bottom wheel surface speed:
         * <pre>
         *   v_surface = (motor_RPS / FLYWHEEL_GEAR_RATIO) × 2π × FLYWHEEL_WHEEL_RADIUS_M
         *   v_ball    = v_surface × FLYWHEEL_EFFICIENCY
         * </pre>
         */

        /** Contact radius of the bottom flywheel wheels (4 in diameter → 2 in radius). */
        public static final double FLYWHEEL_WHEEL_RADIUS_M = Units.inchesToMeters(2.0); // 0.0508 m

        /**
         * Ball-to-flywheel energy transfer efficiency (dimensionless, 0–1).
         * {@code v_ball = v_flywheel_surface × FLYWHEEL_EFFICIENCY}.
         *
         * <p>This is an empirical lump factor that accounts for all contact losses:
         * <ul>
         *   <li><b>Contact area:</b> a larger contact patch (more wheels, softer
         *       compliance, deeper ball compression) provides more friction force and
         *       transfers more momentum — larger contact area → higher efficiency.</li>
         *   <li><b>Slip:</b> any relative sliding between wheel surface and ball
         *       surface dissipates energy as heat.  Running the flywheel faster than
         *       the ball's exit speed (i.e. compression mode) keeps the wheel driving
         *       rather than being driven, but excess speed difference wastes energy.</li>
         *   <li><b>Material:</b> polyurethane compliance wheels on foam balls are
         *       typically 0.75–0.90.  Steel inertia wheels on the same shaft do not
         *       contact the ball and do not affect this value.</li>
         * </ul>
         *
         * <p>Tune by comparing physics-model predicted RPM against actual shooter
         * RPM at a known distance.  Decrease if the ball overshoots; increase if it
         * falls short (after confirming the distance measurement is accurate).
         */
        public static final double FLYWHEEL_EFFICIENCY = 0.745;


        // --- Flywheel Slot 0 PIDF (Velocity control, VelocityVoltage) --------
        public static final double FLYWHEEL_KP = 0.40;
        public static final double FLYWHEEL_KI = 0.00;
        public static final double FLYWHEEL_KD = 0.00;
        /** Feedforward gain in V·s/rot. */
        public static final double FLYWHEEL_KV = 0.12;
        /** Static friction compensation in V. */
        public static final double FLYWHEEL_KS = 0.05;
        /** Acceleration feedforward in V·s²/rot. */
        public static final double FLYWHEEL_KA = 0.01;

        // --- Hood Slot 0 PIDF (Position, MotionMagicVoltage) -----------------
        // Gains are scaled from the GR=2 baseline to GR=21:
        //   kP/kD scale by old_GR/new_GR (same mechanism stiffness in motor-rotation space)
        //   kV/kS/kA unchanged (motor-shaft properties, gear-ratio-independent)
        // TODO: verify on robot — kP and kD especially may need fine-tuning.
        public static final double HOOD_KP = 40.0;   // was 24.0 at GR=2  →  24.0 × (2/21)
        public static final double HOOD_KI = 0.00;
        public static final double HOOD_KD = 0.00;  // was  0.40 at GR=2  →  0.40 × (2/21)
        public static final double HOOD_KV = 0.12;
        public static final double HOOD_KS = 0.25;
        public static final double HOOD_KA = 0.01;

        /**
         * Gravity feedforward for the hood pivot (V).
         *
         * <p>The hood adjusts launch elevation over a narrow 15° range
         * ({@link #HOOD_MIN_ANGLE_DEG} to {@link #HOOD_MAX_ANGLE_DEG}).  Because the
         * travel is small, gravity torque is nearly constant across the full range
         * (cos varies only from ~0.89 to ~0.74 over the physical angles).  Phoenix 6's
         * {@code GravityTypeValue.Elevator_Static} therefore models it correctly: kG is
         * added as a constant voltage in the positive (upward) direction every loop,
         * regardless of current position or velocity.
         *
         * <p>Effect: going up takes the same motor effort as going down because the PID
         * no longer has to generate extra output to fight gravity.  This eliminates the
         * asymmetry where raising the hood (against gravity) required 8 POV presses but
         * lowering it (gravity-assisted) took only 2–3 presses.
         *
         * <p>Start at 0.40 V.  Increase if the hood still struggles going up; decrease
         * if it overshoots or creeps upward when no command is active.
         * TODO: tune on robot.
         */
        public static final double HOOD_KG = 0.09; // TODO: tune on robot

        // --- Hood MotionMagic Profile ----------------------------------------
        // Scaled from GR=2 baseline to GR=21 to maintain the same mechanism angular speed:
        //   motor_vel = mechanism_vel × gear_ratio  →  scale by new_GR/old_GR = 21/2 = 10.5
        // TODO: verify on robot — raise or lower to taste after kP/kD are confirmed.
        /** Cruise velocity of the hood MotionMagic profile in motor rot/s.
         *  33 motor RPS → 226 °/s mechanism (2× original). */
        public static final double HOOD_MM_CRUISE_VEL_RPS   = 33.0;
        /** Acceleration of the hood MotionMagic profile in motor rot/s².
         *  550 → 3771 °/s² mechanism (2.5× original).
         *  A 2° SOTM update settles within ~1 loop at this acceleration. */
        public static final double HOOD_MM_ACCEL_RPSS        = 550.0;
        /** Jerk limit — 0 disables jerk limiting so the profile hits max
         *  acceleration immediately.  SOTM setpoints shift every loop; the
         *  jerk ramp-up was adding 1-2 loops of lag per update. Soft limits
         *  protect the mechanism on large moves. */
        public static final double HOOD_MM_JERK_RPSS2        = 0.0;

        // --- Hood Mechanical Limits ------------------------------------------
        /** Shallowest allowed hood angle in degrees (long-range flat trajectory). */
        public static final double HOOD_MIN_ANGLE_DEG = 27.5;
        /** Steepest allowed hood angle in degrees (short-range lofted trajectory). */
        public static final double HOOD_MAX_ANGLE_DEG = 42.5;

        // --- Current Limits --------------------------------------------------
        public static final double FLYWHEEL_STATOR_LIMIT_A = 70.0;
        public static final double FLYWHEEL_SUPPLY_LIMIT_A  = 70.0;
        public static final double HOOD_STATOR_LIMIT_A      = 70.0;
        public static final double HOOD_SUPPLY_LIMIT_A      = 70.0;

        // --- Readiness Tolerances --------------------------------------------
        /** Flywheel speed tolerance in rot/s for the "at-speed" check. */
        public static final double FLYWHEEL_TOLERANCE_RPS = 2.0;
        /** Hood angle tolerance in degrees for the "at-angle" check. */
        public static final double HOOD_TOLERANCE_DEG = 1.0;

        /**
         * Wider flywheel tolerance used for the <em>continuous fire</em> gate
         * during shoot-on-the-move.  While the static tolerance
         * ({@link #FLYWHEEL_TOLERANCE_RPS}) gates the initial
         * PREPPING→SHOOTING transition, this wider window is used inside
         * {@code handleShooting()} to decide whether to keep feeding each loop.
         *
         * <p>If the flywheel error exceeds this value, the feeder pauses
         * until the wheel catches up, preventing wasted shots when the
         * setpoint shifts rapidly due to dEff changes while driving.
         *
         * <p>Must be ≥ {@link #FLYWHEEL_TOLERANCE_RPS}.  Increase to allow
         * coarser tracking; decrease for tighter control at the cost of
         * more frequent feeder pauses.
         */
        public static final double FLYWHEEL_MOVING_TOLERANCE_RPS = 4.0;

        /**
         * Wider hood tolerance used for the continuous fire gate during
         * shoot-on-the-move — same idea as
         * {@link #FLYWHEEL_MOVING_TOLERANCE_RPS}.
         *
         * <p>Must be ≥ {@link #HOOD_TOLERANCE_DEG}.
         */
        public static final double HOOD_MOVING_TOLERANCE_DEG = 2.5;

        /**
         * Empirical scalar applied to the lateral lead angle during
         * shoot-on-the-move (dimensionless, 0–2).
         *
         * <p>The physics model computes:
         * <pre>  \u03b4\u03b8 = atan2(-v_lateral \u00d7 t_flight, d_eff)</pre>
         * This gain multiplies the result, allowing field-tuning without
         * modifying the physics model constants.  1.0 = pure physics prediction;
         * &gt;1.0 over-leads (use if balls consistently trail); &lt;1.0 under-leads.
         */
        public static final double SOTM_LEAD_ANGLE_SCALAR = 1.0;

        /**
         * Scales the radial (forward/backward) SOTM distance correction applied to
         * flywheel RPM and hood angle.  1.0 = full physics correction; 0.0 = no
         * correction (same as stationary).  Reduce if RPM changes too aggressively
         * during forward/backward movement.
         */
        public static final double SOTM_RADIAL_SCALE = 1.0;

        /**
         * Chassis speed deadband for shoot-on-the-move compensation (m/s).
         *
         * <p>When the total robot translation speed is below this threshold the
         * SOTM pipeline is bypassed entirely: setpoints are computed at the raw
         * vision distance with zero lead angle.
         * This eliminates noise-level jitter from the EMA filter, d_eff
         * correction, and lead angle when the robot is effectively stationary.
         *
         * <p>With EMA filtering (SOTM_VELOCITY_ALPHA), the effective noise floor is
         * ~0.01–0.03 m/s.  Keep at 0.5 m/s until vRadial sign is confirmed on
         * hardware — activating SOTM at low speeds with a wrong vRadial sign
         * produces steep "mountain" trajectories from artificially small dEff.
         */
        public static final double SOTM_SPEED_DEADBAND_MPS = 0.05;

        /**
         * Prediction horizon (seconds) used by the SOTM pipeline to predict the
         * robot's position/velocity when the next ball exits the barrel.
         *
         * <p>Set to one robot loop (20 ms) rather than the full mechanical transit
         * time (~80 ms) because in continuous-fire mode the feeder is always running:
         * a ball can exit at any moment, so the correct horizon is "time until the
         * next loop" — not "time from a cold feeder start."  The system recomputes
         * setpoints every 20 ms, so by the time any ball exits the motors are
         * already at the setpoint computed one loop ago (~20 ms prior), which is
         * essentially real-time.  Using 80 ms would always compute 80 ms into the
         * future, producing setpoints that are stale at the moment of exit.
         *
         * <p>For the first ball after a cold feeder start the prediction is 70 ms
         * short, but that single-ball error is small vs. the gains in continuous-fire
         * accuracy for all subsequent balls.
         */
        public static final double FEEDER_TRANSIT_SECONDS = 0.02; // one robot loop

        /**
         * EMA smoothing factor for chassis velocity in the SOTM pipeline.
         * y_n = α·x_n + (1−α)·y_{n−1}.  Time constant τ = −dt / ln(1−α).
         * At dt=20ms: α=0.40 → τ≈39ms.
         * Higher α = faster response but more noise; lower α = smoother but more lag.
         * Tune on field: increase if SOTM feels sluggish, decrease if dEff jitters.
         */
        public static final double SOTM_VEL_ALPHA = 0.40;

        /**
         * EMA smoothing factor for the finite-difference acceleration estimate.
         * Differentiation amplifies noise so acceleration needs heavier smoothing.
         * At dt=20ms: α=0.25 → τ≈72ms.
         * Note: jerk is intentionally not estimated — at T=FEEDER_TRANSIT_SECONDS
         * (0.02s) the jerk contribution to velocity is ½·j·T²≈0.002 m/s for typical
         * FRC accelerations, completely negligible.
         */
        public static final double SOTM_ACCEL_ALPHA = 0.25;

        // --- Launch Geometry -------------------------------------------------
        /** Height of the flywheel contact point above the floor in meters. */
        public static final double LAUNCH_HEIGHT_M = Units.inchesToMeters(16.5);

        /**
         * Virtual rim buffer added above the HUB rim in trajectory calculations.
         *
         * <p>In {@link frc.robot.util.ShooterKinematics}, the required clearance height is:
         * <pre>
         *   h_clear = HUB_TOP_OPENING_HEIGHT_M + FUEL_RADIUS_M + RIM_SAFETY_MARGIN_M
         * </pre>
         * {@code FUEL_RADIUS_M} accounts for the ball's physical size (center must be at
         * least one radius above the rim).  This constant is the <em>additional</em> buffer
         * on top of that — do NOT include {@code FUEL_RADIUS_M} here again.
         *
         * <p>The solver's tolerance-robust speed step already pads for ±FLYWHEEL_TOLERANCE_RPS
         * and ±HOOD_TOLERANCE_DEG.  This margin is a small geometric buffer only.
         * Increase if first shots clip the near rim; decrease if balls bounce off the back wall.
         */
        public static final double RIM_SAFETY_MARGIN_M = 0.05; // 5 cm — tunable on field
    }

    // =========================================================================
    // Turret Constants
    // =========================================================================

    /** Constants for the Kraken X44–powered turret mechanism. */
    public static final class Turret {

        /** CAN ID of the Kraken X44 turret motor on the CANivore bus. */
        public static final int TURRET_CAN_ID = 22;

        /** Turret gear ratio: 200 : 19 from motor shaft to ring gear. */
        public static final double TURRET_GEAR_RATIO = 200.0 / 19.0;

        /**
         * Encoder angle (in mechanism degrees) of the cable-home position, where the
         * torsion spring is at equilibrium (zero spring torque).
         */
        public static final double TURRET_CABLE_HOME_ENCODER_DEG = -90.0;

        // --- Slot 0 PIDF (Position, MotionMagicVoltage) ----------------------
        public static final double TURRET_KP = 44.0;
        public static final double TURRET_KI = 0.00;
        public static final double TURRET_KD = 1.20;
        public static final double TURRET_KV = 0.10;
        public static final double TURRET_KS = 0.20;
        public static final double TURRET_KA = 0.01;

        // --- MotionMagic Profile ---------------------------------------------
        public static final double TURRET_MM_CRUISE_VEL_RPS = 30.0;
        public static final double TURRET_MM_ACCEL_RPSS      = 300.0;
        public static final double TURRET_MM_JERK_RPSS2      = 0.0;

        // --- Soft Limits (encoder degrees from forward-facing zero) ----------
        /** Maximum encoder position (motor CCW / turret CW) before soft limit engages. */
        public static final double TURRET_FORWARD_LIMIT_DEG =  60.0;
        /** Minimum encoder position (motor CW / turret CCW) before soft limit engages. */
        public static final double TURRET_REVERSE_LIMIT_DEG = -300.0;

        // --- Current Limits --------------------------------------------------
        public static final double TURRET_STATOR_LIMIT_A = 70.0;
        public static final double TURRET_SUPPLY_LIMIT_A  = 70.0;

        // --- Readiness Tolerance ---------------------------------------------
        /** Turret alignment tolerance in degrees for the static "aligned" check
         *  (used by {@link #isReadyToShoot()} before the first shot). */
        public static final double TURRET_TOLERANCE_DEG = 1.0;

        /**
         * Minimum setpoint change that triggers a new motor command when already
         * within {@link #TURRET_TOLERANCE_DEG}.  Vision jitter causes ±0.2–0.4°
         * setpoint noise every loop; below this threshold the motor silently holds
         * its last position command instead of chasing noise.
         */
        public static final double TURRET_SETPOINT_HYSTERESIS_DEG = 0.5;

        /**
         * Wider turret tolerance used during continuous fire while moving
         * (mirrors {@link Shooter#FLYWHEEL_MOVING_TOLERANCE_RPS} /
         * {@link Shooter#HOOD_MOVING_TOLERANCE_DEG}).
         *
         * <p>While shooting on the move the setpoint updates every 20 ms; the
         * turret is always slightly behind the moving target.  A 1° gate would
         * cut the feeder every loop the turret lags even slightly, producing
         * intermittent fire.  This wider window keeps the feeder running as long
         * as the turret is reasonably on-target; accuracy impact is small because
         * the lead-angle compensation already accounts for motion.
         */
        public static final double TURRET_MOVING_TOLERANCE_DEG = 3.0;

        /**
         * Static angular trim added to every turret command (degrees, positive = CCW / left).
         *
         * <p>Compensates for systematic aiming errors that cannot be calibrated out
         * mechanically — e.g. encoder-zero offset, PhotonVision heading bias, or
         * unmeasured turret-pivot misalignment.  With the Limelight disabled, all
         * turret targeting relies on odometry-derived angles; even a 1–2° heading
         * error in the fused pose causes consistent misses.
         *
         * <p>Tune on the field: if all shots land to the <b>right</b> of the HUB,
         * increase this value (push turret left / CCW).  If shots land to the
         * <b>left</b>, decrease (push turret right / CW).
         */
        public static final double TURRET_AIM_TRIM_DEG = 0.0; // positive = CCW (left) correction

        /**
         * Look-ahead time for turret targeting while the robot is rotating (seconds).
         *
         * <p>When the robot spins, the hub's robot-relative angle changes at −ω rad/s.
         * The total latency from measuring the hub angle to the turret physically
         * arriving at the commanded position is roughly:
         *   odometry update ~10 ms + loop delay ~10 ms + motor response ~60–80 ms ≈ 80–100 ms.
         * Without prediction the turret always commands the hub's <em>current</em>
         * position, but by the time it arrives the hub has moved
         * {@code ω × latency} degrees further — at 3 rad/s that is ~17°.
         *
         * <p>The fix: add {@code ω × TURRET_PREDICTION_S} degrees to every hub-angle
         * command so the turret aims where the hub <em>will be</em>, not where it is now.
         *
         * <p>Tune: if the turret overshoots during spin, decrease; if it still lags,
         * increase.  Typical range 0.08–0.15 s.
         */
        public static final double TURRET_PREDICTION_S = 0.10;

        /**
         * Turret pivot offset from the robot's geometric center in robot-relative
         * coordinates (X = forward, Y = left), in meters.  The shooter is mounted
         * at the back-right corner of the robot frame, so both values are negative.
         *
         * <p>Measure from the robot's geometric center to the turret pivot point
         * during final assembly and update both values accordingly.
         */
        public static final double TURRET_OFFSET_X_M = -0.15; // rearward from center
        public static final double TURRET_OFFSET_Y_M = -0.15; // rightward from center

        /**
         * Vulcan spring feedforward gain (Volts per degree) used in
         * {@link frc.robot.subsystems.TurretSubsystem#setAngle(double)}.
         * The spring applies a restoring torque proportional to turret angle;
         * this feedforward counteracts it so the MotionMagic controller does not
         * have to compensate through position error alone.
         *
         * <p>Tune on robot: command the turret to ±90° and increase until
         * steady-state position error drops to near zero.
         */
        public static final double TURRET_SPRING_KF = 0.033; // V/deg — scaled 0.05 × (10/15); TODO: tune on robot
    }

    // =========================================================================
    // Feeder Constants
    // =========================================================================

    /** Constants for the ball feeder that delivers FUEL into the shooter. */
    public static final class Feeder {

        public static final int FEEDER_CAN_ID = 12;

        /** Feeder gear ratio: 2 : 1 from motor to belt/wheel. */
        public static final double FEEDER_GEAR_RATIO = 2.0;

        public static final double FEEDER_FORWARD_PERCENT = 0.80;
        public static final double FEEDER_REVERSE_PERCENT = -0.50;

        // --- Current Limits --------------------------------------------------
        public static final double FEEDER_STATOR_LIMIT_A = 70.0;
        public static final double FEEDER_SUPPLY_LIMIT_A  = 70.0;

        // --- Anti-Jam Detection ----------------------------------------------
        /** Stator current (amps) above which a jam is suspected in the feeder. */
        public static final double FEEDER_JAM_CURRENT_A  = 55.0;
        /** Duration the jam current must persist before triggering the exhaust cycle. */
        public static final double FEEDER_JAM_DURATION_S = 0.3;
        /** Duration of the exhaust (reversal) cycle to clear a detected jam. */
        public static final double FEEDER_EXHAUST_DURATION_S = 0.5;
    }

    // =========================================================================
    // Spindexer Constants
    // =========================================================================

    /** Constants for the spinning disk that queues FUEL for the feeder. */
    public static final class Spindexer {

        public static final int SPINDEXER_CAN_ID = 14;

        /** Spindexer gear ratio: 5 : 1 from motor to disk. */
        public static final double SPINDEXER_GEAR_RATIO = 5.0;

        public static final double SPINDEXER_FORWARD_PERCENT = -0.80;
        public static final double SPINDEXER_REVERSE_PERCENT = 0.40;

        // --- Current Limits --------------------------------------------------
        public static final double SPINDEXER_STATOR_LIMIT_A = 70.0;
        public static final double SPINDEXER_SUPPLY_LIMIT_A  = 70.0;

        // --- Anti-Jam Detection ----------------------------------------------
        /** Stator current (amps) above which a jam is suspected in the spindexer. */
        public static final double SPINDEXER_JAM_CURRENT_A  = 50.0;
        public static final double SPINDEXER_JAM_DURATION_S = 0.3;
        public static final double SPINDEXER_EXHAUST_DURATION_S = 0.5;
    }

    // =========================================================================
    // Intake Constants
    // =========================================================================

    /** Constants for the ground intake (deploy pivot + roller). */
    public static final class Intake {

        // CAN IDs
        /** Master deploy motor (left side). */
        public static final int DEPLOY_LEFT_CAN_ID  = 24;
        /** Follower deploy motor (right side). */
        public static final int DEPLOY_RIGHT_CAN_ID = 25;
        public static final int ROLLER_CAN_ID        = 26;

        // --- Gear Ratios -----------------------------------------------------
        /** Deploy pivot gear ratio: 4 : 1 from motor to pivot joint. */
        public static final double DEPLOY_GEAR_RATIO = 4.0;
        /** Roller gear ratio: 1.5 : 1 from motor to roller contact surface. */
        public static final double ROLLER_GEAR_RATIO = 1.5;

        // --- Deploy Slot 0 PIDF (Position, MotionMagicVoltage) ---------------
        public static final double DEPLOY_KP = 20.0;
        public static final double DEPLOY_KI = 0.00;
        public static final double DEPLOY_KD = 0.30;
        public static final double DEPLOY_KV = 0.12;
        public static final double DEPLOY_KS = 0.20;
        public static final double DEPLOY_KA = 0.01;

        /**
         * Peak gravity-feedforward voltage applied to the deploy arm (volts).
         *
         * <p>The arm is <b>vertical when stowed (0°)</b> and approximately
         * <b>47.5° from vertical when deployed</b>.  Gravity torque on the arm
         * is therefore proportional to {@code sin(sensor_angle)}, not cosine.
         * CTRE's built-in {@code Arm_Cosine} type is NOT used; instead, this
         * constant is applied manually as:
         * <pre>  feedForward = DEPLOY_KG × sin(armAngleDeg)</pre>
         *
         * <p>At stowed (0°): sin(0) = 0 → no feedforward (arm is balanced on the vertical).
         * At deployed (47.5°): sin(47.5°) ≈ 0.737 → 73.7% of DEPLOY_KG applied.
         * Tune by slowly increasing until the arm holds steady at 47.5° with no
         * position error under load.
         */
        public static final double DEPLOY_KG = 0.50; // TODO: tune on robot

        /**
         * Extra feedforward voltage (V) added on top of the gravity feedforward
         * when the arm is stowing (lifting from deployed → vertical).
         *
         * <p>Two factors make stowing the power-hungry direction:
         * <ol>
         *   <li><b>Gravity + mass</b> — the intake is heavy; the arm lifts both its
         *       own mass and the roller assembly against gravity.</li>
         *   <li><b>Open flap</b> — the intake flap is <em>closed</em> when stowed and
         *       <em>open</em> when deployed.  Stowing while the flap is open adds
         *       mechanical resistance (the flap acts against the closing motion)
         *       on top of the gravitational load.</li>
         * </ol>
         * Increase if the arm stalls or cannot complete the stow motion.
         * Decrease if the stow motion overshoots or oscillates at the top.
         */
        public static final double DEPLOY_KG_STOW_EXTRA_V = 2.5; // TODO: tune on robot

        // --- Deploy MotionMagic Profile — asymmetric per direction -----------
        //
        // Deploying (stowed → deployed, arm falls with gravity):
        //   Use slow cruise velocity — gravity assists and the arm would overshoot
        //   or impact the field at high speed without a tight velocity limit.
        //
        // Stowing (deployed → stowed, arm lifts against gravity):
        //   Use a higher cruise velocity so the motors have time to generate peak
        //   torque and overcome gravity + roller assembly weight.

        /** Cruise velocity (motor rot/s) when deploying (arm falling with gravity). Slow. */
        public static final double DEPLOY_MM_CRUISE_VEL_DEPLOY_RPS = 2.0; // TODO: tune
        /** Cruise velocity (motor rot/s) when stowing (arm lifting against gravity). Faster. */
        public static final double DEPLOY_MM_CRUISE_VEL_STOW_RPS   = 5.0; // TODO: tune

        public static final double DEPLOY_MM_ACCEL_RPSS  = 10.0;
        public static final double DEPLOY_MM_JERK_RPSS2  = 100.0;

        // --- Deploy Position Setpoints (mechanism degrees from vertical = 0°) -
        /** Stowed position: arm vertical, perpendicular to ground (0°). */
        public static final double DEPLOY_STOWED_DEG   = 0.0;
        /** Deployed position: arm ~47.5° from vertical, nearly parallel to ground. */
        public static final double DEPLOY_DEPLOYED_DEG = 47.5;

        // --- Deploy Position Tolerances --------------------------------------

        /**
         * Tolerance (degrees) for the deployed-position check ({@code isDeployed()}).
         *
         * <p>The intake is mechanically heavy and carries a spring-loaded flap.
         * MotionMagic will settle within a few degrees of the setpoint rather than
         * exactly on it.  This tolerance defines "close enough to deploy and collect
         * balls".  Tighten only if intake performance noticeably degrades at the
         * boundary; loosen if the mechanism never registers as deployed.
         * TODO: verify on the physical robot.
         */
        public static final double DEPLOY_TOLERANCE_DEG = 4.0;

        /**
         * Tolerance (degrees) for the stowed-position check ({@code isStowed()}).
         *
         * <p>A wider tolerance than {@link #DEPLOY_TOLERANCE_DEG} is appropriate here:
         * stowing against gravity + an open flap means the arm may not reach exactly 0°
         * before the control loop is satisfied.  The TRENCH clearance check and the
         * Superstructure use this to confirm the arm is safely retracted.
         * TODO: verify TRENCH clearance with the arm at this angle before each event.
         */
        public static final double STOW_TOLERANCE_DEG = 5.0;

        // --- Roller Physical Dimensions --------------------------------------
        /** Roller contact diameter (2 in). */
        public static final double ROLLER_DIAMETER_M = Units.inchesToMeters(2.0);

        /**
         * Kraken X60 free-spin speed at 12 V (CTRE datasheet: ≈ 6 000 RPM).
         * Used only for computing the intake drive-speed cap; no closed-loop
         * control relies on this value.
         */
        public static final double KRAKEN_FREE_SPEED_RPS = 100.0;

        /**
         * Empirical fraction of free-spin speed the roller achieves under typical
         * intake load.  Open-loop duty-cycle control loses speed under torque;
         * for a lightly loaded ball intake this is roughly 0.35–0.55 × free speed.
         * <p>Tune this value on the field: decrease if the robot still pushes balls,
         * increase if the drive cap feels unnecessarily restrictive.
         */
        public static final double ROLLER_LOAD_FACTOR = 0.45;

        // --- Roller Percent Output -------------------------------------------
        public static final double ROLLER_INTAKE_PERCENT  = 0.80;
        public static final double ROLLER_EXHAUST_PERCENT = -0.50;

        /**
         * Maximum drivetrain <em>translation</em> speed (m/s) while the intake is
         * deployed.  Keeping drive speed below the roller contact surface speed
         * (adjusted for motor load) ensures the roller grips and pulls the ball
         * in rather than pushing it away.
         *
         * <pre>
         *   v_max = ROLLER_INTAKE_PERCENT × KRAKEN_FREE_SPEED_RPS
         *           / ROLLER_GEAR_RATIO × π × ROLLER_DIAMETER_M × ROLLER_LOAD_FACTOR
         *         ≈ 0.80 × 100 / 1.5 × π × 0.0508 × 0.45  ≈  3.8 m/s
         * </pre>
         */
        public static final double MAX_DRIVE_SPEED_WHILE_INTAKING_MPS =
                ROLLER_INTAKE_PERCENT
                * KRAKEN_FREE_SPEED_RPS
                / ROLLER_GEAR_RATIO
                * Math.PI
                * ROLLER_DIAMETER_M
                * ROLLER_LOAD_FACTOR;

        // --- Current Limits --------------------------------------------------
        public static final double DEPLOY_LEFT_STATOR_LIMIT_A  = 70.0;
        public static final double DEPLOY_LEFT_SUPPLY_LIMIT_A  = 70.0;
        public static final double DEPLOY_RIGHT_STATOR_LIMIT_A = 70.0;
        public static final double DEPLOY_RIGHT_SUPPLY_LIMIT_A = 70.0;
        public static final double ROLLER_STATOR_LIMIT_A  = 70.0;
        public static final double ROLLER_SUPPLY_LIMIT_A  = 70.0;

        /**
         * Inversion for the left (master) deploy motor.
         * Positive sensor direction = arm deploying downward.
         */
        public static final InvertedValue DEPLOY_LEFT_INVERT = InvertedValue.Clockwise_Positive;

        /**
         * Inversion for the right (follower) deploy motor hardware config.
         *
         * <p><b>Note:</b> when a motor is in {@link com.ctre.phoenix6.controls.Follower}
         * mode, Phoenix 6 <em>ignores</em> {@code MotorOutputConfigs.Inverted} entirely.
         * Direction in follower mode is controlled exclusively by
         * {@code Follower.opposeDirection} ({@link #DEPLOY_FOLLOWER_OPPOSES_MASTER}).
         * This constant is kept only so the motor has a sensible inversion if ever
         * taken out of follower mode (e.g. during SysId characterization).
         */
        public static final InvertedValue DEPLOY_RIGHT_INVERT = InvertedValue.CounterClockwise_Positive;

        /**
         * Whether the right (follower) deploy motor opposes the master's direction.
         *
         * <p>Must be {@code true}: the two motors are mirror-mounted on opposite sides
         * of the pivot, so the right motor must spin in the opposite direction of the
         * left master to produce the same physical arm motion.  In Phoenix 6 Follower
         * mode the hardware {@code Inverted} config is bypassed, so this flag is the
         * only way to achieve the required counter-rotation.
         */
        public static final MotorAlignmentValue DEPLOY_FOLLOWER_OPPOSES_MASTER = MotorAlignmentValue.Opposed;
    }

    // =========================================================================
    // Vision Constants (Camera Hardware)
    // =========================================================================

    /** Camera hardware constants for the Limelight-based AprilTag vision system. */
    public static final class Vision {

        /** NetworkTables hostname of the Limelight used for HUB targeting. */
        public static final String LIMELIGHT_NAME = "limelight";

        /**
         * Height of the Limelight camera lens above the floor in meters.
         * Measure from carpet to optical center after final robot assembly.
         */
        public static final double CAMERA_HEIGHT_M = Units.inchesToMeters(20.0);

        /**
         * Upward pitch of the Limelight camera from horizontal in degrees.
         * Positive = tilted upward.  Calibrate with a target at a known distance.
         */
        public static final double CAMERA_PITCH_DEG = 30.0;

        /**
         * Minimum acceptable pose-estimation confidence score (0–1) for a tag to be
         * used as the primary aiming reference.
         */
        public static final double MIN_TAG_CONFIDENCE = 0.5;

        /**
         * Maximum acceptable single-tag pose ambiguity ratio for HUB targeting.
         * Limelight reports a value in [0, 1]; values above this threshold indicate
         * the solver could not distinguish between two mirror-image pose solutions
         * and the tag is skipped for aiming purposes.
         *
         * <p>0.15 is the standard Limelight recommendation for reliable disambiguation.
         */
        public static final double MAX_TAG_AMBIGUITY = 0.15;
    }

    // =========================================================================
    // PhotonVision Corner Camera Constants
    // =========================================================================

    /**
     * Constants for the four corner-mounted PhotonVision cameras.
     *
     * <p>Each camera sits at a corner of the robot chassis at 45° yaw to each
     * drivetrain side, angled slightly upward so AprilTags are visible under the
     * TRENCH (the robot is built low).  All four cameras use a 80° FOV lens.
     *
     * <p>Per-camera enable flags let individual cameras be disabled in software
     * when physically missing or misconfigured without redeploying all code.
     */
    public static final class PhotonVisionConstants {

        // --- Per-camera enable flags -----------------------------------------
        // Set false for any camera that is not physically installed or connected.

        /** Front-Left corner camera enable. Set {@code false} to disable. */
        public static final boolean CAMERA_FL_ENABLED = true;
        /** Front-Right corner camera enable. Set {@code false} to disable. */
        public static final boolean CAMERA_FR_ENABLED = true;
        /** Back-Left corner camera enable. Set {@code false} to disable. */
        public static final boolean CAMERA_BL_ENABLED = true;
        /** Back-Right corner camera enable. Set {@code false} to disable. */
        public static final boolean CAMERA_BR_ENABLED = true;

        // --- Camera names (must match PhotonVision server pipeline names) -----

        /** PhotonVision pipeline name for the Front-Left camera. */
        public static final String CAMERA_FL_NAME = "photon_fl";
        /** PhotonVision pipeline name for the Front-Right camera. */
        public static final String CAMERA_FR_NAME = "photon_fr";
        /** PhotonVision pipeline name for the Back-Left camera. */
        public static final String CAMERA_BL_NAME = "photon_bl";
        /** PhotonVision pipeline name for the Back-Right camera. */
        public static final String CAMERA_BR_NAME = "photon_br";

        // --- Camera optics ---------------------------------------------------

        /** Horizontal FOV of all four PhotonVision cameras in degrees (80° lens). */
        public static final double CAMERA_FOV_DEG = 80.0;

        // --- Camera pose relative to robot center (X forward, Y left, Z up) --
        //
        // Positive X → forward from robot center
        // Positive Y → left from robot center
        // Positive Z → up from carpet

        /** Half-length (X) of the robot chassis from center to front/rear edge. */
        private static final double FRAME_HALF_X_M = Units.inchesToMeters(14.5);

        /** Half-width (Y) of the robot chassis from center to left/right edge. */
        private static final double FRAME_HALF_Y_M = Units.inchesToMeters(14.5);

        /** Camera optical center height above carpet. */
        public static final double CAMERA_HEIGHT_M = Units.inchesToMeters(8.0);

        /**
         * Upward tilt angle of each camera from horizontal (degrees).
         * All four cameras share the same tilt angle since the robot is symmetric
         * and the TRENCH height constraint is the same in all directions.
         */
        public static final double CAMERA_PITCH_UP_DEG = 20.0;

        // --- Robot-to-camera Transform3d definitions -------------------------
        //
        // Each Transform3d describes the camera's position and orientation
        // relative to the robot center.
        //
        // Yaw values (CCW-positive viewed from above):
        //   FL = +45°  (points forward-left)
        //   FR = -45°  (points forward-right)
        //   BL = +135° (points backward-left)
        //   BR = -135° (points backward-right)
        //
        // Pitch = -CAMERA_PITCH_UP_DEG: in WPILib, a negative pitch around the Y
        // axis tilts the camera's nose (Z-axis) upward in field coordinates.

        /** Robot-center → Front-Left camera optical center. */
        public static final edu.wpi.first.math.geometry.Transform3d ROBOT_TO_CAMERA_FL =
            new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(
                    FRAME_HALF_X_M, FRAME_HALF_Y_M, CAMERA_HEIGHT_M),
                new edu.wpi.first.math.geometry.Rotation3d(
                    0,
                    -Units.degreesToRadians(CAMERA_PITCH_UP_DEG),
                    Units.degreesToRadians(45.0)));

        /** Robot-center → Front-Right camera optical center. */
        public static final edu.wpi.first.math.geometry.Transform3d ROBOT_TO_CAMERA_FR =
            new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(
                    FRAME_HALF_X_M, -FRAME_HALF_Y_M, CAMERA_HEIGHT_M),
                new edu.wpi.first.math.geometry.Rotation3d(
                    0,
                    -Units.degreesToRadians(CAMERA_PITCH_UP_DEG),
                    Units.degreesToRadians(-45.0)));

        /** Robot-center → Back-Left camera optical center. */
        public static final edu.wpi.first.math.geometry.Transform3d ROBOT_TO_CAMERA_BL =
            new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(
                    -FRAME_HALF_X_M, FRAME_HALF_Y_M, CAMERA_HEIGHT_M),
                new edu.wpi.first.math.geometry.Rotation3d(
                    0,
                    -Units.degreesToRadians(CAMERA_PITCH_UP_DEG),
                    Units.degreesToRadians(135.0)));

        /** Robot-center → Back-Right camera optical center. */
        public static final edu.wpi.first.math.geometry.Transform3d ROBOT_TO_CAMERA_BR =
            new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(
                    -FRAME_HALF_X_M, -FRAME_HALF_Y_M, CAMERA_HEIGHT_M),
                new edu.wpi.first.math.geometry.Rotation3d(
                    0,
                    -Units.degreesToRadians(CAMERA_PITCH_UP_DEG),
                    Units.degreesToRadians(-135.0)));

        // --- Pose-estimation quality filters ---------------------------------

        /**
         * Maximum single-tag pose ambiguity ratio [0–1].
         * Estimates with ambiguity above this value are discarded unless ≥2 tags
         * are visible (multi-tag PnP is inherently unambiguous).
         * 0.2 is more permissive than the Limelight recommendation of 0.15 because
         * corner cameras may have worse lighting conditions.
         */
        public static final double MAX_AMBIGUITY = 0.2;

        /**
         * Maximum rotation rate above which vision measurements are rejected
         * (degrees per second).  High angular velocity causes motion blur and
         * makes time-stamp latency corrections inaccurate.
         */
        public static final double MAX_ROTATION_RATE_DEG_PER_S = 720.0;

        // --- Standard deviations for pose-estimator weighting ----------------
        //
        // YAGSL-style heuristic: separate base values for single-tag and
        // multi-tag estimates, then scaled by (1 + avgDist² / 30).
        // Lower = trust the vision measurement more relative to odometry.
        // Units: meters for X/Y, radians for theta.

        /** Single-tag base X/Y std dev (m).  Relatively high because single-tag
         *  PnP has an ambiguity axis that can flip the solution. */
        public static final double SINGLE_TAG_XY_STD_DEV_M = 4.0;

        /** Single-tag base theta std dev (rad).  MAX_VALUE = never correct heading
         *  from cameras; the Pigeon 2 gyro is the sole heading source (same
         *  policy as Limelight MegaTag2).  Camera-mount calibration errors in
         *  ROBOT_TO_CAMERAS would otherwise bias the fused heading continuously. */
        public static final double SINGLE_TAG_THETA_STD_DEV_RAD = Double.MAX_VALUE;

        /** Multi-tag base X/Y std dev (m).  Multi-tag PnP eliminates the
         *  ambiguity problem, so we trust it significantly more. */
        public static final double MULTI_TAG_XY_STD_DEV_M = 0.5;

        /** Multi-tag base theta std dev (rad).  MAX_VALUE = never correct heading
         *  from cameras; gyro only.  See SINGLE_TAG_THETA_STD_DEV_RAD comment. */
        public static final double MULTI_TAG_THETA_STD_DEV_RAD = Double.MAX_VALUE;

        // --- False-pose rejection thresholds ---------------------------------

    }

    // =========================================================================
    // Superstructure Constants
    // =========================================================================

    /** High-level constants governing the robot's overall state machine. */
    public static final class SuperstructureConstants {

        /**
         * Safety margin subtracted from the 3-second HUB pulsing window.
         * We stop firing this many seconds before the 3-second grace period expires
         * to avoid a ball being in-flight when the HUB goes inactive.
         */
        public static final double HUB_PULSE_SAFETY_MARGIN_S = 0.5;

        /** Maximum shooting range (meters).  Shots beyond this distance are inhibited. */
        public static final double MAX_SHOOT_RANGE_M = 7.0;

        /** Minimum shooting range (meters).  Shots inside this distance are inhibited. */
        public static final double MIN_SHOOT_RANGE_M = 1.5;

        /**
         * Default number of balls loaded into the robot at match start.
         * Exposed on SmartDashboard as "Preload Ball Count" so it can be
         * adjusted from the driver station without redeploying code.
         */
        public static final int PRELOAD_BALL_COUNT = 3;

        // Alliance-wall pass setpoints are computed dynamically in ShootCommand
        // using ShooterKinematics.calculate(distanceToWall) so RPM and hood angle
        // scale correctly with the robot's distance from the wall.

    }

    // =========================================================================
    // Trench Constants (TE-26200 field element)
    // =========================================================================

    /**
     * Constants describing the TRENCH field element (TE-26200) for the 2026 REBUILT game.
     *
     * <p>Dimensions reference:
     * <ul>
     *   <li>TE-26200 — Overall TRENCH structure assembly</li>
     *   <li>TE-26204 — Interior clearance envelope (robot traversal corridor)</li>
     * </ul>
     */
    public static final class TrenchConstants {

        // --- Overall Structure Dimensions (TE-26200) -------------------------

        /** Total width of the TRENCH structure parallel to robot travel in meters (65.65 in). */
        public static final double TRENCH_TOTAL_WIDTH_M  = Units.inchesToMeters(65.65);

        /** Total depth of the TRENCH structure perpendicular to robot travel in meters (47.0 in). */
        public static final double TRENCH_TOTAL_DEPTH_M  = Units.inchesToMeters(47.0);

        /** Total height of the TRENCH structure from carpet in meters (40.25 in). */
        public static final double TRENCH_TOTAL_HEIGHT_M = Units.inchesToMeters(40.25);

        // --- Clearance Envelope (TE-26204) ------------------------------------

        /** Clear width of the robot transit corridor inside the TRENCH in meters (50.34 in). */
        public static final double TRENCH_CLEARANCE_WIDTH_M  = Units.inchesToMeters(50.34);

        /**
         * Maximum interior clear height from carpet to the lowest overhead beam in
         * meters (22.125 in per TE-26204).
         */
        public static final double TRENCH_CLEARANCE_HEIGHT_M = Units.inchesToMeters(22.125);

        /**
         * Maximum allowed robot height to safely transit the TRENCH in meters.
         * One inch of safety margin is subtracted from the clearance height.
         * Value: 22.125 − 1.0 = 21.125 in ≈ 0.537 m.
         */
        public static final double TRENCH_MAX_ROBOT_HEIGHT_M =
                TRENCH_CLEARANCE_HEIGHT_M - Units.inchesToMeters(1.0);

        /**
         * Horizontal distance from the TRENCH edge at which the robot should begin
         * stowing all overhead mechanisms to guarantee clearance on entry.
         */
        public static final double TRENCH_APPROACH_STOW_DISTANCE_M = 1.5;

        // --- AprilTag Heights (TRENCH tags) -----------------------------------

        /**
         * Height of TRENCH AprilTag centers above carpet on the official competition
         * field in meters (35.0 in).
         * Source: 2026 REBUILT field specifications — verify before each event.
         */
        public static final double TRENCH_APRILTAG_HEIGHT_OFFICIAL_M =
                Units.inchesToMeters(35.0);

        /**
         * Height of TRENCH AprilTag centers above carpet on the TE-26300 practice
         * field in meters (35.6875 in).
         * Derivation: bottom edge = 31.875 in; tag size = 7.625 in;
         * center = 31.875 + 7.625/2 = 35.6875 in.
         * Delta vs. official: +0.6875 in.
         */
        public static final double TRENCH_APRILTAG_HEIGHT_PRACTICE_M =
                Units.inchesToMeters(35.6875);

        /**
         * Resolved TRENCH AprilTag height for the current venue, controlled by
         * {@link VisionConstants#IS_ANDYMARK_FIELD}.
         */
        public static final double ACTIVE_TRENCH_APRILTAG_HEIGHT_M =
                VisionConstants.IS_ANDYMARK_FIELD
                        ? TRENCH_APRILTAG_HEIGHT_PRACTICE_M
                        : TRENCH_APRILTAG_HEIGHT_OFFICIAL_M;

        // --- AprilTag IDs (TRENCH structures) --------------------------------
        //
        // Blue-alliance TRENCH: tags 1, 6, 7, 12
        // Red-alliance  TRENCH: tags 17, 22, 23, 28

        /** All TRENCH AprilTag IDs (both alliances). */
        public static final int[] TRENCH_ALL_TAG_IDS = { 1, 6, 7, 12, 17, 22, 23, 28 };

        /** Blue alliance TRENCH AprilTag IDs. */
        public static final int[] TRENCH_BLUE_TAG_IDS = { 1, 6, 7, 12 };

        /** Red alliance TRENCH AprilTag IDs. */
        public static final int[] TRENCH_RED_TAG_IDS = { 17, 22, 23, 28 };
    }

    // =========================================================================
    // Outpost Constants
    // =========================================================================

    /**
     * Physical and timing constants for the OUTPOST structures.
     *
     * <p>There are 2 OUTPOSTs — one per ALLIANCE WALL, each in the right corner
     * relative to that alliance's driver station.  The CHUTE accepts up to
     * {@link #OUTPOST_CHUTE_CAPACITY} FUEL from the HUMAN PLAYER.  The base
     * opening (32 in × 7 in, 1.88 in off floor) is where ROBOTS dock to receive
     * the dispensed FUEL.
     */
    public static final class OutpostConstants {

        /**
         * Width of the OUTPOST structure measured along the field-width (Y) axis
         * (49.32 in).  The OUTPOST center Y = OUTPOST_WIDTH_M / 2 from the
         * nearest boundary wall.
         */
        public static final double OUTPOST_WIDTH_M = Units.inchesToMeters(49.32); // 1.253 m

        /**
         * Distance from the alliance wall to the robot's center when docked in
         * front of the OUTPOST base opening.  Adjust if the robot frame is shorter
         * or longer than assumed.
         */
        public static final double OUTPOST_DOCK_DISTANCE_FROM_WALL_M = 0.7; // m — must be ≥0.6 m to clear the navgrid wall obstacle zone (2 × 0.3 m nodes)

        /** How long the robot waits at the OUTPOST for the HUMAN PLAYER to open
         *  the CHUTE DOOR and fill the robot (seconds). */
        public static final double OUTPOST_WAIT_SECONDS = 10.0;

        /** Maximum number of FUEL balls the OUTPOST CHUTE can hold at once. */
        public static final int OUTPOST_CHUTE_CAPACITY = 25;

        /** Timeout for the shoot-while-driving phase (seconds).  Generous to
         *  allow all pre-loaded balls to clear even on a long path. */
        public static final double OUTPOST_DRIVE_SHOOT_TIMEOUT_S = 15.0;

        /** Timeout for the shoot phase after receiving OUTPOST fuel (seconds). */
        public static final double OUTPOST_RECEIVE_SHOOT_TIMEOUT_S = 30.0;

        /**
         * How long to wait for PhotonVision to produce a valid pose correction
         * at the start of autonomous before giving up and using raw odometry.
         * 2 seconds is enough for cameras to connect and process the first frame.
         */
        public static final double VISION_POSE_INIT_TIMEOUT_S = 2.0;

        // --- Trench-to-Outpost Auto Constants --------------------------------

        /** Timeout for the shoot-while-driving phase from trench to outpost (seconds). */
        public static final double TRENCH_TO_OUTPOST_DRIVE_SHOOT_TIMEOUT_S = 15.0;
    }

    // =========================================================================
    // Field Layout Constants
    // =========================================================================

    /**
     * Field layout constants for the WPILib coordinate system.
     *
     * <p>WPILib origin is at the bottom-left corner of the field from the Blue Alliance
     * driver-station perspective.  +X = toward Red Alliance wall; +Y = toward the
     * Blue Alliance's left side wall.
     *
     * <p><b>TRENCH center positions are estimates — measure on the actual field and
     * update before each event.</b>
     */
    public static final class FieldLayout {

        // --- Field Dimensions ------------------------------------------------

        /** Total field length in meters (X-axis, Blue to Red wall, 651 in). */
        public static final double FIELD_LENGTH_M = Units.inchesToMeters(651.2); // ~16.54 m

        /** Total field width in meters (Y-axis, 317.7 in per REBUILT game manual). */
        public static final double FIELD_WIDTH_M  = Units.inchesToMeters(317.7); // ~8.07 m

        // --- HUB Centers -----------------------------------------------------

        /**
         * Center of the Blue Alliance HUB in field coordinates.
         * X = distance from the Blue wall to the hub's front face + half the
         * hub base width (so we target the geometric center, not the front rim).
         * Y = field centerline.
         */
        public static final edu.wpi.first.math.geometry.Translation2d BLUE_HUB_CENTER =
                new edu.wpi.first.math.geometry.Translation2d(
                        Field.HUB_DIST_FROM_ALLIANCE_WALL_M + (Field.HUB_BASE_WIDTH_M / 2.0),
                        FIELD_WIDTH_M / 2.0);

        /**
         * Center of the Red Alliance HUB in field coordinates.
         * Symmetric with {@link #BLUE_HUB_CENTER} across the field midpoint.
         */
        public static final edu.wpi.first.math.geometry.Translation2d RED_HUB_CENTER =
                new edu.wpi.first.math.geometry.Translation2d(
                        FIELD_LENGTH_M - (Field.HUB_DIST_FROM_ALLIANCE_WALL_M + (Field.HUB_BASE_WIDTH_M / 2.0)),
                        FIELD_WIDTH_M / 2.0);

        // --- Pass Targets --------------------------------------------------------
        //
        // When passing from the natural zone the ball should land where an alliance
        // partner can collect it.  The target is a 2-D field point chosen based on
        // which side of the hub the robot is on (split at Y = FIELD_WIDTH_M / 2).
        //
        // Right side (robot Y < FIELD_WIDTH_M/2, near bottom/right wall):
        //   X = midpoint(alliance wall, hub front face)
        //   Y = midpoint(bottom wall, hub right face)
        //
        // Left side (robot Y >= FIELD_WIDTH_M/2, near top/left wall):
        //   X = midpoint(alliance wall, hub geometric center)
        //   Y = midpoint(hub left face, top wall)
        //
        // Red targets are mirrored in X across the field centre.

        /** Blue pass target — robot on right (bottom-wall) side of hub. */
        public static final edu.wpi.first.math.geometry.Translation2d BLUE_PASS_TARGET_RIGHT =
                new edu.wpi.first.math.geometry.Translation2d(
                        Field.HUB_DIST_FROM_ALLIANCE_WALL_M / 2.0,
                        (FIELD_WIDTH_M / 2.0 - Field.HUB_BASE_WIDTH_M / 2.0) / 2.0);

        /** Blue pass target — robot on left (top-wall) side of hub. */
        public static final edu.wpi.first.math.geometry.Translation2d BLUE_PASS_TARGET_LEFT =
                new edu.wpi.first.math.geometry.Translation2d(
                        (Field.HUB_DIST_FROM_ALLIANCE_WALL_M + Field.HUB_BASE_WIDTH_M / 2.0) / 2.0,
                        (FIELD_WIDTH_M / 2.0 + Field.HUB_BASE_WIDTH_M / 2.0 + FIELD_WIDTH_M) / 2.0);

        /** Red pass target — robot on right (bottom-wall) side of hub. */
        public static final edu.wpi.first.math.geometry.Translation2d RED_PASS_TARGET_RIGHT =
                new edu.wpi.first.math.geometry.Translation2d(
                        FIELD_LENGTH_M - Field.HUB_DIST_FROM_ALLIANCE_WALL_M / 2.0,
                        (FIELD_WIDTH_M / 2.0 - Field.HUB_BASE_WIDTH_M / 2.0) / 2.0);

        /** Red pass target — robot on left (top-wall) side of hub. */
        public static final edu.wpi.first.math.geometry.Translation2d RED_PASS_TARGET_LEFT =
                new edu.wpi.first.math.geometry.Translation2d(
                        FIELD_LENGTH_M - (Field.HUB_DIST_FROM_ALLIANCE_WALL_M + Field.HUB_BASE_WIDTH_M / 2.0) / 2.0,
                        (FIELD_WIDTH_M / 2.0 + Field.HUB_BASE_WIDTH_M / 2.0 + FIELD_WIDTH_M) / 2.0);

        // --- TRENCH Bounding Box Centers (ESTIMATES — verify on field) --------

        /**
         * Center positions of TRENCH structures on the Blue Alliance half of the field.
         * Index 0 = bottom-wall TRENCH; index 1 = top-wall TRENCH.
         *
         * <p>Y: the TRENCH is flush against the bottom/top boundary wall, so the
         * center is exactly {@code TRENCH_TOTAL_WIDTH_M / 2} from each wall
         * (≈ 0.834 m). WIDTH is the Y-axis dimension (spanning field width).
         * Top TRENCH is symmetric: {@code FIELD_WIDTH_M - TRENCH_TOTAL_WIDTH_M / 2}.
         * X: same as the alliance HUB center X (TRENCH is centered on the hub).
         */
        public static final edu.wpi.first.math.geometry.Translation2d[] TRENCH_BLUE_CENTERS = {
            new edu.wpi.first.math.geometry.Translation2d(BLUE_HUB_CENTER.getX(), TrenchConstants.TRENCH_TOTAL_WIDTH_M / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(BLUE_HUB_CENTER.getX(), FIELD_WIDTH_M - TrenchConstants.TRENCH_TOTAL_WIDTH_M / 2.0),
        };

        /**
         * Center positions of TRENCH structures on the Red Alliance half of the field.
         * Index 0 = bottom-wall TRENCH; index 1 = top-wall TRENCH.
         *
         * <p>Y values are identical to {@link #TRENCH_BLUE_CENTERS} — the TRENCH
         * structures span the full field width symmetrically.
         * X: center along the TRENCH run — measure on field and update as needed.
         */
        public static final edu.wpi.first.math.geometry.Translation2d[] TRENCH_RED_CENTERS = {
            new edu.wpi.first.math.geometry.Translation2d(RED_HUB_CENTER.getX(), TrenchConstants.TRENCH_TOTAL_WIDTH_M / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(RED_HUB_CENTER.getX(), FIELD_WIDTH_M - TrenchConstants.TRENCH_TOTAL_WIDTH_M / 2.0),
        };

        // --- Hub Approach Poses ----------------------------------------------

        /**
         * Robot pose for hub-alignment approach on the Blue Alliance.
         * Positioned 3.5 m in front of the Blue HUB, facing the hub (0°).
         */
        public static final edu.wpi.first.math.geometry.Pose2d BLUE_HUB_APPROACH_POSE =
                new edu.wpi.first.math.geometry.Pose2d(
                        BLUE_HUB_CENTER.getX() - 3.5,
                        BLUE_HUB_CENTER.getY(),
                        new edu.wpi.first.math.geometry.Rotation2d(0));

        /**
         * Robot pose for hub-alignment approach on the Red Alliance.
         * Positioned 3.5 m in front of the Red HUB, facing the hub (180°).
         */
        public static final edu.wpi.first.math.geometry.Pose2d RED_HUB_APPROACH_POSE =
                new edu.wpi.first.math.geometry.Pose2d(
                        RED_HUB_CENTER.getX() + 3.5,
                        RED_HUB_CENTER.getY(),
                        edu.wpi.first.math.geometry.Rotation2d.k180deg);

        // --- Outpost Dock Poses -----------------------------------------------

        /**
         * Robot docking pose at the Blue Alliance OUTPOST.
         *
         * <p>The OUTPOST sits in the right corner relative to the Blue driver
         * station (Y = {@code OUTPOST_WIDTH_M / 2} from the bottom boundary wall).
         * The robot faces the Blue wall (180°) so its intake/funnel aligns with
         * the base opening that dispenses FUEL.
         */
        public static final edu.wpi.first.math.geometry.Pose2d BLUE_OUTPOST_DOCK_POSE =
                new edu.wpi.first.math.geometry.Pose2d(
                        OutpostConstants.OUTPOST_DOCK_DISTANCE_FROM_WALL_M,
                        OutpostConstants.OUTPOST_WIDTH_M / 2.0,
                        edu.wpi.first.math.geometry.Rotation2d.k180deg);

        /**
         * Robot docking pose at the Red Alliance OUTPOST.
         *
         * <p>Symmetric with {@link #BLUE_OUTPOST_DOCK_POSE} by 180° rotation
         * around the field center (right corner from Red driver-station
         * perspective).  Robot faces the Red wall (0°).
         */
        public static final edu.wpi.first.math.geometry.Pose2d RED_OUTPOST_DOCK_POSE =
                new edu.wpi.first.math.geometry.Pose2d(
                        FIELD_LENGTH_M - OutpostConstants.OUTPOST_DOCK_DISTANCE_FROM_WALL_M,
                        FIELD_WIDTH_M - OutpostConstants.OUTPOST_WIDTH_M / 2.0,
                        new edu.wpi.first.math.geometry.Rotation2d(0));

        // --- Trench Through-Poses --------------------------------------------
        //
        // These are the hub-side exit poses for each TRENCH structure.  The X
        // coordinate is placed just past the hub-side structural edge of the trench
        // (trench center X ± half-width − buffer), so PathPlanner is forced to
        // route through the trench opening rather than around it.
        //
        // The robot faces the hub after exiting (0° for Blue, 180° for Red).
        // Depth along the robot travel axis (X): TrenchConstants.TRENCH_TOTAL_DEPTH_M.
        // Buffer of 0.3 m is added past the edge for clearance before PathPlanner
        // hands off the second pathfind segment.
        //
        // Index 0 = bottom-wall trench; index 1 = top-wall trench.
        // <b>Y values match TRENCH_BLUE_CENTERS / TRENCH_RED_CENTERS — update together.</b>

        private static final double TRENCH_EXIT_BUFFER_M = 0.3;

        /** Hub-side exit poses for the two Blue Alliance TRENCH structures. */
        public static final edu.wpi.first.math.geometry.Pose2d[] BLUE_TRENCH_THROUGH_POSES = {
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_BLUE_CENTERS[0].getX()
                    - TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0 - TRENCH_EXIT_BUFFER_M,
                TRENCH_BLUE_CENTERS[0].getY(),
                new edu.wpi.first.math.geometry.Rotation2d(0)),
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_BLUE_CENTERS[1].getX()
                    - TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0 - TRENCH_EXIT_BUFFER_M,
                TRENCH_BLUE_CENTERS[1].getY(),
                new edu.wpi.first.math.geometry.Rotation2d(0)),
        };

        /** Hub-side exit poses for the two Red Alliance TRENCH structures. */
        public static final edu.wpi.first.math.geometry.Pose2d[] RED_TRENCH_THROUGH_POSES = {
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_RED_CENTERS[0].getX()
                    + TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0 + TRENCH_EXIT_BUFFER_M,
                TRENCH_RED_CENTERS[0].getY(),
                edu.wpi.first.math.geometry.Rotation2d.k180deg),
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_RED_CENTERS[1].getX()
                    + TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0 + TRENCH_EXIT_BUFFER_M,
                TRENCH_RED_CENTERS[1].getY(),
                edu.wpi.first.math.geometry.Rotation2d.k180deg),
        };

        /**
         * Neutral-zone-side exit poses for the two Blue Alliance TRENCH structures.
         * Placed just past the outpost-side (high-X) structural edge of the trench.
         * Robot faces the neutral zone (0°) after exiting.
         */
        public static final edu.wpi.first.math.geometry.Pose2d[] BLUE_TRENCH_NEUTRAL_THROUGH_POSES = {
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_BLUE_CENTERS[0].getX()
                    + TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0 + TRENCH_EXIT_BUFFER_M,
                TRENCH_BLUE_CENTERS[0].getY(),
                new edu.wpi.first.math.geometry.Rotation2d(0)),
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_BLUE_CENTERS[1].getX()
                    + TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0 + TRENCH_EXIT_BUFFER_M,
                TRENCH_BLUE_CENTERS[1].getY(),
                new edu.wpi.first.math.geometry.Rotation2d(0)),
        };

        /**
         * Neutral-zone-side exit poses for the two Red Alliance TRENCH structures.
         * Placed just past the outpost-side (low-X) structural edge of the trench.
         * Robot faces the neutral zone (180°) after exiting.
         */
        public static final edu.wpi.first.math.geometry.Pose2d[] RED_TRENCH_NEUTRAL_THROUGH_POSES = {
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_RED_CENTERS[0].getX()
                    - TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0 - TRENCH_EXIT_BUFFER_M,
                TRENCH_RED_CENTERS[0].getY(),
                edu.wpi.first.math.geometry.Rotation2d.k180deg),
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_RED_CENTERS[1].getX()
                    - TrenchConstants.TRENCH_TOTAL_DEPTH_M / 2.0 - TRENCH_EXIT_BUFFER_M,
                TRENCH_RED_CENTERS[1].getY(),
                edu.wpi.first.math.geometry.Rotation2d.k180deg),
        };

        /**
         * Neutral zone target poses for the Blue Alliance.
         * One pose per TRENCH (indexed same as TRENCH_BLUE_CENTERS).
         * Placed at the field midpoint X, aligned with each trench's Y center.
         * Robot faces the neutral zone / opponent side (0°).
         */
        public static final edu.wpi.first.math.geometry.Pose2d[] BLUE_NEUTRAL_ZONE_POSES = {
            new edu.wpi.first.math.geometry.Pose2d(
                FIELD_LENGTH_M / 2.0,
                TRENCH_BLUE_CENTERS[0].getY(),
                new edu.wpi.first.math.geometry.Rotation2d(0)),
            new edu.wpi.first.math.geometry.Pose2d(
                FIELD_LENGTH_M / 2.0,
                TRENCH_BLUE_CENTERS[1].getY(),
                new edu.wpi.first.math.geometry.Rotation2d(0)),
        };

        /**
         * Neutral zone target poses for the Red Alliance.
         * One pose per TRENCH (indexed same as TRENCH_RED_CENTERS).
         * Placed at the field midpoint X, aligned with each trench's Y center.
         * Robot faces the neutral zone / opponent side (180°).
         */
        public static final edu.wpi.first.math.geometry.Pose2d[] RED_NEUTRAL_ZONE_POSES = {
            new edu.wpi.first.math.geometry.Pose2d(
                FIELD_LENGTH_M / 2.0,
                TRENCH_RED_CENTERS[0].getY(),
                edu.wpi.first.math.geometry.Rotation2d.k180deg),
            new edu.wpi.first.math.geometry.Pose2d(
                FIELD_LENGTH_M / 2.0,
                TRENCH_RED_CENTERS[1].getY(),
                edu.wpi.first.math.geometry.Rotation2d.k180deg),
        };
    }
}
