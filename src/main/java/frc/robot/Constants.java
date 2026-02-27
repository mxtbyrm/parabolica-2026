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

        // CAN IDs (on "canivore" bus)
        public static final int FLYWHEEL_CAN_ID = 10;
        public static final int HOOD_CAN_ID     = 11;

        // --- Beam-break sensor (roboRIO DIO) ---------------------------------
        /**
         * DIO port of the beam-break sensor mounted between the feeder exit and
         * the flywheel contact zone.  The beam is broken (LOW) while a ball is
         * passing; the rising edge (ball clears the beam) is used to count each
         * shot and decrement the ball counter.
         *
         * <p>Update this port to match the physical wiring once installed.
         */
        public static final int SHOOTER_BEAM_BREAK_DIO_PORT = 0;

        // --- Gear Ratios (motor rotations : mechanism rotations) -------------

        /** Flywheel gear ratio: 1.5 : 1 from motor shaft to flywheel contact wheel. */
        public static final double FLYWHEEL_GEAR_RATIO = 1.5;

        /** Hood gear ratio: 2 : 1 from motor shaft to hood pivot. */
        public static final double HOOD_GEAR_RATIO = 2.0;

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
        public static final double HOOD_KP = 24.0;
        public static final double HOOD_KI = 0.00;
        public static final double HOOD_KD = 0.40;
        public static final double HOOD_KV = 0.12;
        public static final double HOOD_KS = 0.25;
        public static final double HOOD_KA = 0.01;

        // --- Hood MotionMagic Profile ----------------------------------------
        // Conservative values for initial PID tuning: ~5% of Kraken free speed.
        // Raise after gains are verified on the real mechanism.
        /** Cruise velocity of the hood MotionMagic profile in motor rot/s. */
        public static final double HOOD_MM_CRUISE_VEL_RPS   = 5.0;
        /** Acceleration of the hood MotionMagic profile in motor rot/s². */
        public static final double HOOD_MM_ACCEL_RPSS        = 10.0;
        /** Jerk limit of the hood MotionMagic profile in motor rot/s³. */
        public static final double HOOD_MM_JERK_RPSS2        = 100.0;

        // --- Hood Mechanical Limits ------------------------------------------
        /** Shallowest allowed hood angle in degrees (long-range flat trajectory). */
        public static final double HOOD_MIN_ANGLE_DEG = 20.0;
        /** Steepest allowed hood angle in degrees (short-range lofted trajectory). */
        public static final double HOOD_MAX_ANGLE_DEG = 65.0;

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

        /** CAN ID of the Kraken X44 turret motor on the canivore bus. */
        public static final int TURRET_CAN_ID = 12;

        // TODO: VERIFY BEFORE DEPLOYING — believed to be 16:1 or 24:1.
        // Using 24.0 as the conservative placeholder (higher ratio → less mechanism
        // travel per motor rotation → safer if wrong). Measure motor-to-ring-gear
        // tooth counts and update this value before running turret commands.
        public static final double TURRET_GEAR_RATIO = 24.0;

        // --- Slot 0 PIDF (Position, MotionMagicVoltage) ----------------------
        public static final double TURRET_KP = 24.0;
        public static final double TURRET_KI = 0.00;
        public static final double TURRET_KD = 0.50;
        public static final double TURRET_KV = 0.12;
        public static final double TURRET_KS = 0.20;
        public static final double TURRET_KA = 0.01;

        // --- MotionMagic Profile ---------------------------------------------
        // Conservative values for initial PID tuning: ~5% of Kraken free speed.
        // Raise after gear ratio is confirmed and gains are verified on the real mechanism.
        public static final double TURRET_MM_CRUISE_VEL_RPS = 5.0;
        public static final double TURRET_MM_ACCEL_RPSS      = 10.0;
        public static final double TURRET_MM_JERK_RPSS2      = 100.0;

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
         * Approximate distance from the robot's rotation center to the turret muzzle
         * in meters.  Used by {@link frc.robot.commands.ShootCommand} to compute the
         * tangential muzzle velocity from robot spin (ω × r).
         *
         * <p>Measure from the robot's geometric center to the exit point of the ball
         * during final assembly and update this value accordingly.
         */
        public static final double TURRET_RADIUS_FROM_CENTER_M = 0.25; // ~10 in — measure on robot
    }

    // =========================================================================
    // Feeder Constants
    // =========================================================================

    /** Constants for the ball feeder that delivers FUEL into the shooter. */
    public static final class Feeder {

        public static final int FEEDER_CAN_ID = 13;

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
        public static final int DEPLOY_LEFT_CAN_ID  = 15;
        /** Follower deploy motor (right side). */
        public static final int DEPLOY_RIGHT_CAN_ID = 16;
        public static final int ROLLER_CAN_ID        = 17;

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
         * Gravity feedforward for the rotating deploy arm (GravityType = Arm_Cosine).
         * Phoenix 6 automatically scales this by cos(mechanism_angle), so it applies
         * full voltage at 0° (horizontal) and zero at 90° (vertical, gravity-neutral).
         * Tune by slowly increasing until the arm holds horizontal without position error.
         */
        public static final double DEPLOY_KG = 0.30;

        // --- Deploy MotionMagic Profile --------------------------------------
        // Conservative values for initial PID tuning: ~5% of Kraken free speed.
        // Raise after gains are verified on the real mechanism.
        public static final double DEPLOY_MM_CRUISE_VEL_RPS = 5.0;
        public static final double DEPLOY_MM_ACCEL_RPSS      = 10.0;
        public static final double DEPLOY_MM_JERK_RPSS2      = 100.0;

        // --- Deploy Position Setpoints (mechanism degrees from stowed = 0°) --
        public static final double DEPLOY_STOWED_DEG   = 0.0;
        public static final double DEPLOY_DEPLOYED_DEG = 90.0;

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
        public static final InvertedValue DEPLOY_LEFT_INVERT = InvertedValue.CounterClockwise_Positive;

        /**
         * Inversion for the right (follower) deploy motor.
         * The right motor is mirror-mounted on the opposite side of the robot, so it
         * must spin clockwise-positive to produce the same downward arm motion as the
         * left motor spinning counter-clockwise-positive.
         * Applied via {@code MotorOutputConfigs.Inverted}; the {@link com.ctre.phoenix6.controls.Follower}
         * request uses {@code opposeDirection = false} and lets the hardware config drive inversion.
         */
        public static final InvertedValue DEPLOY_RIGHT_INVERT = InvertedValue.Clockwise_Positive;

        /**
         * Motor alignment for the right (follower) deploy motor.
         * {@link MotorAlignmentValue#Aligned} because inversion is already handled by
         * {@link #DEPLOY_RIGHT_INVERT} via {@code MotorOutputConfigs.Inverted}; the
         * {@link com.ctre.phoenix6.controls.Follower} request does not need to flip
         * direction on top of that.
         */
        public static final MotorAlignmentValue DEPLOY_RIGHT_OPPOSES_MASTER = MotorAlignmentValue.Aligned;
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
