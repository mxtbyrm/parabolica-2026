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
 * <h2>Practice vs. Official Field</h2>
 * <p>Set {@link VisionConstants#IS_PRACTICE_FIELD} to {@code true} at a practice venue.
 * That single flag propagates to both HUB and TRENCH AprilTag heights automatically.
 */
public final class Constants {

    private Constants() {} // Utility class — do not instantiate.

    // =========================================================================
    // VisionConstants — Practice Field Toggle
    // =========================================================================

    /**
     * Vision-related field-configuration constants.
     *
     * <p><b>IS_PRACTICE_FIELD is the single source of truth for venue selection.</b>
     * It controls AprilTag height resolution for both HUB and TRENCH:
     * <ul>
     *   <li>HUB tags:    Official = 44.25 in | Practice = 45.0625 in (+0.8125 in)</li>
     *   <li>TRENCH tags: Official = 35.0 in  | Practice = 35.6875 in (+0.6875 in)</li>
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
         * Master practice-field toggle. Set {@code true} at a TE-26300 practice venue;
         * {@code false} at an official competition field.
         *
         * <p>This flag is read by {@link Field#ACTIVE_HUB_APRILTAG_HEIGHT_M} and
         * {@link TrenchConstants#ACTIVE_TRENCH_APRILTAG_HEIGHT_M} to select the
         * correct AprilTag height for each venue.
         */
        public static final boolean IS_PRACTICE_FIELD = false;
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

        /** Width of the hexagonal HUB top opening across flats. */
        public static final double HUB_TOP_OPENING_DIAMETER_M =
                Units.inchesToMeters(41.7);   // 1.0592 m

        /** Side length of the square HUB base footprint (47.0 in). */
        public static final double HUB_BASE_WIDTH_M =
                Units.inchesToMeters(47.0);   // 1.1938 m

        /** Distance from the alliance wall to the HUB center (158.6 in). */
        public static final double HUB_DIST_FROM_ALLIANCE_WALL_M =
                Units.inchesToMeters(158.6);  // 4.0284 m

        // --- HUB Geometry — Practice Field (TE-26300) ------------------------

        /**
         * Height of the HUB AprilTag centers above carpet on the TE-26300 practice field.
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

        /**
         * Delegates to {@link VisionConstants#IS_PRACTICE_FIELD}.
         * Use {@code VisionConstants.IS_PRACTICE_FIELD} directly in new code;
         * this alias exists for backward compatibility.
         */
        public static final boolean IS_PRACTICE_FIELD = VisionConstants.IS_PRACTICE_FIELD;

        /**
         * Resolved AprilTag height for the current venue, selected by
         * {@link VisionConstants#IS_PRACTICE_FIELD}.  All vision distance calculations
         * should reference this constant (or use
         * {@link frc.robot.subsystems.VisionSubsystem#getTagHeightMeters(int)} for
         * per-tag routing).
         */
        public static final double ACTIVE_HUB_APRILTAG_HEIGHT_M =
                VisionConstants.IS_PRACTICE_FIELD
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

        /** Distance from Alliance Wall to HUB center (158.6 in). */
        public static final double HUB_DIST_FROM_ALLIANCE_WALL_M =
                Units.inchesToMeters(158.6); // 4.0284 m

        /** Width of the hexagonal top opening across flats (41.7 in). */
        public static final double HUB_TOP_OPENING_WIDTH_M =
                Units.inchesToMeters(41.7);  // 1.0592 m

        /** Height of the front rim of the top opening above carpet (72.0 in). */
        public static final double HUB_TOP_OPENING_HEIGHT_M =
                Units.inchesToMeters(72.0);  // 1.8288 m

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
         * {@link VisionConstants#IS_PRACTICE_FIELD}.
         */
        public static final double ACTIVE_HUB_APRILTAG_HEIGHT_M =
                VisionConstants.IS_PRACTICE_FIELD
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
        public static final int FLYWHEEL_CAN_ID = 20;
        public static final int HOOD_CAN_ID     = 21;

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
        public static final int    SHOOTER_CANRANGE_CAN_ID      = 20;

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
        public static final double HOOD_GEAR_RATIO = 21.0;

        /**
         * Hood motor inversion.  Motor CCW → belt CCW → gear CCW → hood CW → hood lifts.
         * {@code CounterClockwise_Positive} (Phoenix 6 default) means CCW motor rotation
         * produces a positive encoder reading, so positive position commands increase the
         * hood angle (raise the launch angle) as expected.
         */
        public static final InvertedValue HOOD_INVERT = InvertedValue.Clockwise_Positive;

        /**
         * Flywheel motor inversion. Set to {@code Clockwise_Positive} if the motor must
         * spin clockwise (when viewed from the shaft end) to produce forward ball launch.
         * Flip to {@code CounterClockwise_Positive} if balls launch in reverse or the
         * wheel spins the wrong direction on a positive RPM command.
         */
        public static final InvertedValue FLYWHEEL_INVERT = InvertedValue.CounterClockwise_Positive;

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
        public static final double FLYWHEEL_EFFICIENCY = 0.85;

        // --- Flywheel Slot 0 PIDF (Velocity control, VelocityVoltage) --------
        public static final double FLYWHEEL_KP = 0.10;
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
        public static final double HOOD_KP = 2.3;   // was 24.0 at GR=2  →  24.0 × (2/21)
        public static final double HOOD_KI = 0.00;
        public static final double HOOD_KD = 0.04;  // was  0.40 at GR=2  →  0.40 × (2/21)
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
        public static final double HOOD_KG = 0.22; // TODO: tune on robot

        // --- Hood MotionMagic Profile ----------------------------------------
        // Scaled from GR=2 baseline to GR=21 to maintain the same mechanism angular speed:
        //   motor_vel = mechanism_vel × gear_ratio  →  scale by new_GR/old_GR = 21/2 = 10.5
        // TODO: verify on robot — raise or lower to taste after kP/kD are confirmed.
        /** Cruise velocity of the hood MotionMagic profile in motor rot/s. */
        public static final double HOOD_MM_CRUISE_VEL_RPS   = 52.0;  // was 5.0  at GR=2
        /** Acceleration of the hood MotionMagic profile in motor rot/s². */
        public static final double HOOD_MM_ACCEL_RPSS        = 100.0; // was 10.0 at GR=2
        /** Jerk limit of the hood MotionMagic profile in motor rot/s³. */
        public static final double HOOD_MM_JERK_RPSS2        = 1000.0; // was 100.0 at GR=2

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
         * Maximum rate at which the flywheel setpoint may <em>decrease</em> while
         * ShootCommand is active (RPM per second).
         *
         * <p>When the robot drives toward the HUB, the effective distance correction
         * ({@code d_eff = d_raw − v_radial × t_flight}) causes the desired flywheel
         * RPM to drop by hundreds of RPM.  If the setpoint drops faster than the
         * flywheel can shed speed, {@code isFlywheelAtSpeed()} never returns true and
         * the robot cannot fire while approaching.  This limit keeps the commanded RPM
         * within a range the flywheel can actually track, accepting a small transient
         * accuracy penalty in exchange for reliable shoot-while-moving capability.
         *
         * <p>Spin-up (setpoint increase) is intentionally unlimited so the flywheel
         * reaches temperature quickly on command start.
         */
        public static final double FLYWHEEL_SLEW_RATE_DOWN_RPM_PER_S = 300.0;

        // --- Launch Geometry -------------------------------------------------
        /** Height of the flywheel contact point above the floor in meters. */
        public static final double LAUNCH_HEIGHT_M = Units.inchesToMeters(24.0);

        /**
         * Pure geometric safety buffer added above the HUB rim in trajectory calculations.
         *
         * <p>In {@link frc.robot.util.ShooterKinematics}, the required clearance height is:
         * <pre>
         *   h_clear = HUB_TOP_OPENING_HEIGHT_M + FUEL_RADIUS_M + RIM_SAFETY_MARGIN_M
         * </pre>
         * {@code FUEL_RADIUS_M} accounts for the ball's physical size (center must be at
         * least one radius above the rim).  This constant is the <em>additional</em> buffer
         * on top of that — do NOT include {@code FUEL_RADIUS_M} here again.
         */
        public static final double RIM_SAFETY_MARGIN_M =
                Units.inchesToMeters(0.5); // 0.0127 m — pure geometry margin
    }

    // =========================================================================
    // Turret Constants
    // =========================================================================

    /** Constants for the Kraken X44–powered turret mechanism. */
    public static final class Turret {

        /** CAN ID of the Kraken X44 turret motor on the CANivore bus. */
        public static final int TURRET_CAN_ID = 22;

        /** Turret gear ratio: 10 : 1 from motor shaft to ring gear. */
        public static final double TURRET_GEAR_RATIO = 10.0;

        // --- Slot 0 PIDF (Position, MotionMagicVoltage) ----------------------
        // Gains are scaled from the GR=24 baseline to GR=10:
        //   kP/kD scale by old_GR/new_GR (same mechanism stiffness in motor-rotation space)
        //   kV/kS/kA unchanged (motor-shaft properties, gear-ratio-independent)
        // TODO: verify on robot — kP and kD especially may need fine-tuning.
        public static final double TURRET_KP = 58.0;  // was 24.0 at GR=24  →  24.0 × (24/10)
        public static final double TURRET_KI = 0.00;
        public static final double TURRET_KD = 1.2;   // was  0.50 at GR=24  →  0.50 × (24/10)
        public static final double TURRET_KV = 0.12;
        public static final double TURRET_KS = 0.20;
        public static final double TURRET_KA = 0.01;

        // --- MotionMagic Profile ---------------------------------------------
        // Scaled from GR=24 baseline to GR=10 to maintain the same mechanism angular speed:
        //   motor_vel = mechanism_vel × gear_ratio  →  scale by new_GR/old_GR = 10/24 = 0.417
        // TODO: verify on robot — raise cruise velocity once gains are confirmed.
        public static final double TURRET_MM_CRUISE_VEL_RPS = 2.0;  // was 5.0  at GR=24
        public static final double TURRET_MM_ACCEL_RPSS      = 4.0;  // was 10.0 at GR=24
        public static final double TURRET_MM_JERK_RPSS2      = 40.0; // was 100.0 at GR=24

        // --- Soft Limits (mechanism degrees from forward-facing zero) --------
        /** Maximum clockwise turret angle before soft limit engages. */
        public static final double TURRET_FORWARD_LIMIT_DEG = 175.0;
        /** Maximum counter-clockwise turret angle before soft limit engages. */
        public static final double TURRET_REVERSE_LIMIT_DEG = -175.0;

        // --- Current Limits --------------------------------------------------
        public static final double TURRET_STATOR_LIMIT_A = 70.0;
        public static final double TURRET_SUPPLY_LIMIT_A  = 70.0;

        // --- Readiness Tolerance ---------------------------------------------
        /** Turret alignment tolerance in degrees for the "aligned" check. */
        public static final double TURRET_TOLERANCE_DEG = 1.0;

        /**
         * Turret pivot offset from the robot's geometric center in robot-relative
         * coordinates (X = forward, Y = left), in meters.  The shooter is mounted
         * at the back-right corner of the robot frame, so both values are negative.
         *
         * <p>Measure from the robot's geometric center to the turret pivot point
         * during final assembly and update both values accordingly.
         */
        public static final double TURRET_OFFSET_X_M = -0.15; // TODO: measure — rearward from center
        public static final double TURRET_OFFSET_Y_M = -0.15; // TODO: measure — rightward from center

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
        public static final double TURRET_SPRING_KF = 0.05; // V/deg — scaled 0.02 × (24/10); TODO: tune on robot
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

        public static final double SPINDEXER_FORWARD_PERCENT = 0.60;
        public static final double SPINDEXER_REVERSE_PERCENT = -0.40;

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
     *
     * <p><b>TODO: measure all X/Y/Z offsets on the physical robot before
     * the first event and replace the placeholder values below.</b>
     */
    public static final class PhotonVisionConstants {

        // --- Per-camera enable flags -----------------------------------------
        // Set false for any camera that is not physically installed or connected.

        /** Front-Left corner camera enable. Set {@code false} to disable. */
        public static final boolean CAMERA_FL_ENABLED = false;
        /** Front-Right corner camera enable. Set {@code false} to disable. */
        public static final boolean CAMERA_FR_ENABLED = false;
        /** Back-Left corner camera enable. Set {@code false} to disable. */
        public static final boolean CAMERA_BL_ENABLED = false;
        /** Back-Right corner camera enable. Set {@code false} to disable. */
        public static final boolean CAMERA_BR_ENABLED = false;

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
        //
        // TODO: replace placeholder dimensions with measurements from robot CAD /
        //       field calibration.  A tape-measure survey from the robot center to
        //       each camera's optical center is sufficient for competition accuracy.

        /** Half-length (X) of the robot chassis from center to front/rear edge. */
        private static final double FRAME_HALF_X_M = Units.inchesToMeters(14.5); // TODO: measure

        /** Half-width (Y) of the robot chassis from center to left/right edge. */
        private static final double FRAME_HALF_Y_M = Units.inchesToMeters(14.5); // TODO: measure

        /** Camera optical center height above carpet. TODO: measure on robot. */
        public static final double CAMERA_HEIGHT_M = Units.inchesToMeters(8.0);  // TODO: measure

        /**
         * Upward tilt angle of each camera from horizontal (degrees).
         * All four cameras share the same tilt angle since the robot is symmetric
         * and the TRENCH height constraint is the same in all directions.
         * TODO: tune — more tilt helps see high tags; less helps see low tags.
         */
        public static final double CAMERA_PITCH_UP_DEG = 20.0; // TODO: tune

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
        // Lower = trust the vision measurement more relative to odometry.
        // Scale with distance: farther tags → larger std dev.
        // Units: meters for X/Y, radians for theta.

        /** Base X/Y std dev (m) at 1-meter tag distance, single tag. */
        public static final double BASE_XY_STD_DEV_M = 0.5;

        /** Base theta std dev (rad) at 1-meter tag distance, single tag. */
        public static final double BASE_THETA_STD_DEV_RAD = 0.5;

        /**
         * Std-dev scale factor for multi-tag (≥2 tags) estimates.
         * Multi-tag PnP is more accurate so we trust it more (smaller sigma).
         */
        public static final double MULTI_TAG_STD_DEV_SCALE = 0.3;
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

        // --- Inactive-Period Alliance Pass -----------------------------------
        // During the inactive period, operators cannot score in the HUB.  The
        // robot instead passes balls over the neutral zone into the alliance zone
        // using these fixed setpoints.  Tune all three values on the field.

        /**
         * Flywheel speed (RPM) for the alliance-wall pass during the inactive period.
         * Set high enough to loft the ball over the neutral zone.
         * TODO: tune on field.
         */
        public static final double PASS_FLYWHEEL_RPM = 2500.0;

        /**
         * Hood angle (degrees) for the alliance-wall pass.
         * Higher value = more loft = longer range.
         * TODO: tune on field.
         */
        public static final double PASS_HOOD_ANGLE_DEG = 55.0;

        // Note: turret angle during passing is computed dynamically in the shoot
        // commands from the robot's field-relative heading and DriverStation.getAlliance().
        // No constant is needed here.

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
         * {@link VisionConstants#IS_PRACTICE_FIELD}.
         */
        public static final double ACTIVE_TRENCH_APRILTAG_HEIGHT_M =
                VisionConstants.IS_PRACTICE_FIELD
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
        public static final double FIELD_LENGTH_M = Units.inchesToMeters(651.0); // ~16.54 m

        /** Total field width in meters (Y-axis, 323 in). */
        public static final double FIELD_WIDTH_M  = Units.inchesToMeters(323.0); // ~8.20 m

        // --- HUB Centers -----------------------------------------------------

        /**
         * Center of the Blue Alliance HUB in field coordinates.
         * X = {@link Field#HUB_DIST_FROM_ALLIANCE_WALL_M} from the Blue wall;
         * Y = field centerline.
         */
        public static final edu.wpi.first.math.geometry.Translation2d BLUE_HUB_CENTER =
                new edu.wpi.first.math.geometry.Translation2d(
                        Field.HUB_DIST_FROM_ALLIANCE_WALL_M,
                        FIELD_WIDTH_M / 2.0);

        /**
         * Center of the Red Alliance HUB in field coordinates.
         * Symmetric with {@link #BLUE_HUB_CENTER} across the field midpoint.
         */
        public static final edu.wpi.first.math.geometry.Translation2d RED_HUB_CENTER =
                new edu.wpi.first.math.geometry.Translation2d(
                        FIELD_LENGTH_M - Field.HUB_DIST_FROM_ALLIANCE_WALL_M,
                        FIELD_WIDTH_M / 2.0);

        // --- TRENCH Bounding Box Centers (ESTIMATES — verify on field) --------

        /**
         * Center positions of TRENCH structures on the Blue Alliance half of the field.
         * Index 0 = bottom-wall TRENCH; index 1 = top-wall TRENCH.
         * <p><b>These are estimates — measure and update before each competition.</b>
         */
        public static final edu.wpi.first.math.geometry.Translation2d[] TRENCH_BLUE_CENTERS = {
            new edu.wpi.first.math.geometry.Translation2d(7.00, 1.60),
            new edu.wpi.first.math.geometry.Translation2d(7.00, 6.60),
        };

        /**
         * Center positions of TRENCH structures on the Red Alliance half of the field.
         * Index 0 = bottom-wall TRENCH; index 1 = top-wall TRENCH.
         * <p><b>These are estimates — measure and update before each competition.</b>
         */
        public static final edu.wpi.first.math.geometry.Translation2d[] TRENCH_RED_CENTERS = {
            new edu.wpi.first.math.geometry.Translation2d(9.50, 1.60),
            new edu.wpi.first.math.geometry.Translation2d(9.50, 6.60),
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

        // --- Trench Through-Poses --------------------------------------------
        //
        // These are the hub-side exit poses for each TRENCH structure.  The X
        // coordinate is placed just past the hub-side structural edge of the trench
        // (trench center X ± half-width − buffer), so PathPlanner is forced to
        // route through the trench opening rather than around it.
        //
        // The robot faces the hub after exiting (0° for Blue, 180° for Red).
        // Width along the robot travel axis: TrenchConstants.TRENCH_TOTAL_WIDTH_M.
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
                    - TrenchConstants.TRENCH_TOTAL_WIDTH_M / 2.0 - TRENCH_EXIT_BUFFER_M,
                TRENCH_BLUE_CENTERS[0].getY(),
                new edu.wpi.first.math.geometry.Rotation2d(0)),
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_BLUE_CENTERS[1].getX()
                    - TrenchConstants.TRENCH_TOTAL_WIDTH_M / 2.0 - TRENCH_EXIT_BUFFER_M,
                TRENCH_BLUE_CENTERS[1].getY(),
                new edu.wpi.first.math.geometry.Rotation2d(0)),
        };

        /** Hub-side exit poses for the two Red Alliance TRENCH structures. */
        public static final edu.wpi.first.math.geometry.Pose2d[] RED_TRENCH_THROUGH_POSES = {
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_RED_CENTERS[0].getX()
                    + TrenchConstants.TRENCH_TOTAL_WIDTH_M / 2.0 + TRENCH_EXIT_BUFFER_M,
                TRENCH_RED_CENTERS[0].getY(),
                edu.wpi.first.math.geometry.Rotation2d.k180deg),
            new edu.wpi.first.math.geometry.Pose2d(
                TRENCH_RED_CENTERS[1].getX()
                    + TrenchConstants.TRENCH_TOTAL_WIDTH_M / 2.0 + TRENCH_EXIT_BUFFER_M,
                TRENCH_RED_CENTERS[1].getY(),
                edu.wpi.first.math.geometry.Rotation2d.k180deg),
        };
    }
}
