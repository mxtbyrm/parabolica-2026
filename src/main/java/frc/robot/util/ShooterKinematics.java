package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.robot.Constants.Field;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.Turret;

/**
 * Computes flywheel RPM and hood angle from the robot's horizontal distance to
 * the HUB center.
 *
 * <h2>Hood Angle vs. Ball Exit Angle</h2>
 * <p>This is a flywheel shooter: the hood deflects the ball, so the hood angle
 * and ball exit (launch) angle are <em>not</em> the same.  The relationship is:
 * <pre>
 *   ballExitAngle = 90° − hoodAngle
 * </pre>
 * Hood at 90° → ball exits flat (0°).  Hood at 20° → ball exits at 70° (steep).
 * All physics simulation uses the ball exit angle; the solver outputs hood angles.
 *
 * <h2>Physics Model (with aerodynamic drag)</h2>
 * <p>The trajectory is simulated numerically using Euler integration at
 * {@link #SIM_DT_S} time steps.  At each step the drag force is:
 * <pre>
 *   F_drag = ½ · C_d · ρ · A · v²
 *   a_drag = F_drag / m = DRAG_ACCEL_COEFF · v²   (opposing velocity direction)
 * </pre>
 * <h2>3-Point Vacuum Parabola</h2>
 * <p>Rather than sweeping hood angles and picking the minimum RPM, the solver
 * fits a vacuum parabola {@code y = h₀ + b·x + a·x²} through three
 * geometric constraint points:
 * <ol>
 *   <li><b>Shooter exit</b> — {@code (0, LAUNCH_HEIGHT_M)}</li>
 *   <li><b>Front rim clearance</b> — {@code (dRim, hClearance)}</li>
 *   <li><b>Hub center at rim height</b> — {@code (dCenter, HUB_TOP_OPENING_HEIGHT_M)}</li>
 * </ol>
 * This uniquely determines the ball exit angle {@code θ = atan(b)}, which is
 * converted to a hood angle and clamped to the mechanism limits.  A binary
 * search then finds the minimum launch speed at that angle that clears the
 * front rim in the full drag simulation.  The result is the geometrically
 * correct trajectory that arcs over the front rim and descends through the
 * hub center at the hub's geometric center height (58.125 in) — well below
 * the 72 in rim — guaranteed to drop into the basket.
 *
 * <p>Results are precomputed once by {@link #precompute()} into interpolation
 * tables, then looked up via {@link #calculate(double)}.
 *
 * <h2>Hexagonal Rim Geometry</h2>
 * <p>The HUB top opening is hexagonal.  The distance from the opening center
 * to the rim varies with approach angle: minimum at a face midpoint
 * (apothem = across-flats / 2) and maximum at a vertex (circumradius =
 * across-flats / √3).  Because we cannot know the approach angle at
 * precompute time, the solver uses the <em>circumradius</em> (worst-case
 * vertex approach) so the ball clears the rim at any angle.
 *
 * @see #calculate(double)
 * @see #getFlightTimeSeconds(double, ShooterSetpoint)
 */
public final class ShooterKinematics {

    /**
     * Distance step in meters between precomputed table entries.
     * Smaller = more accurate interpolation; larger = faster precompute.
     * 0.05 m gives 111 entries across the 1.5–7.0 m range — negligible memory.
     */
    private static final double PRECOMPUTE_STEP_M = 0.05;

    // =========================================================================
    // Physical Constants
    // =========================================================================

    private static final double GRAVITY_MPS2 = 9.81;

    /** Drag coefficient for a smooth sphere (dimensionless). */
    private static final double DRAG_COEFFICIENT = 0.47;

    /** Air density at sea level in kg/m³. */
    private static final double AIR_DENSITY_KGM3 = 1.225;

    /** Cross-sectional area of the FUEL game piece in m². */
    private static final double BALL_CROSS_AREA_M2 =
            Math.PI * Field.FUEL_RADIUS_M * Field.FUEL_RADIUS_M;

    /**
     * Drag deceleration coefficient in (m/s²)/(m/s)², i.e. drag acceleration per
     * unit of speed squared.
     * <pre>
     *   a_drag = DRAG_ACCEL_COEFF × |v|² / |v|   → decomposed into vx, vy components
     * </pre>
     */
    private static final double DRAG_ACCEL_COEFF =
            (0.5 * DRAG_COEFFICIENT * AIR_DENSITY_KGM3 * BALL_CROSS_AREA_M2)
            / Field.FUEL_MASS_KG;

    // =========================================================================
    // Solver Configuration
    // =========================================================================

    /** Euler integration step size in seconds. Smaller = more accurate, more CPU. */
    private static final double SIM_DT_S = 0.010; // 10 ms

    // (Angle sweep removed — hood angle is now determined analytically
    //  by the 3-point vacuum parabola constraint.)

    /** Binary search iteration count for minimum launch speed. */
    private static final int BINARY_SEARCH_ITERS = 35;

    /**
     * Maximum Euler steps per simulation call (= 50 s at SIM_DT_S = 10 ms).
     * Prevents infinite loops if drag causes vx to approach zero asymptotically.
     */
    private static final int MAX_SIM_STEPS = 5000;

    /** Maximum plausible launch speed in m/s for binary search upper bound. */
    private static final double MAX_LAUNCH_SPEED_MPS = 30.0;

    /**
     * Maximum distance from the center of the hexagonal HUB opening to any
     * rim point (circumradius).  For a regular hexagon with across-flats
     * diameter D: {@code circumradius = D / √3}.
     *
     * <p>Used instead of the apothem ({@code D / 2}) so that trajectory
     * clearance is checked at the worst-case approach angle (through a vertex),
     * where the near rim is farther from the hub center, making the
     * turret-to-rim distance <em>shorter</em>.  This avoids undershooting for
     * approach angles between flat faces.
     */
    private static final double HEX_RIM_CIRCUMRADIUS_M =
            Field.HUB_TOP_OPENING_DIAMETER_M / Math.sqrt(3.0);

    // =========================================================================
    // Interpolation Tables (populated once at robot init by precompute())
    // =========================================================================

    /**
     * Physics-computed RPM table.  Keys = distance to hub in meters; values = flywheel RPM.
     * Populated by {@link #precompute()} at robot startup; empty until then.
     */
    private static final InterpolatingDoubleTreeMap RPM_TABLE = new InterpolatingDoubleTreeMap();

    /**
     * Physics-computed hood-angle table.  Keys = distance to hub in meters;
     * values = hood angle in degrees (= 90° − ballExitAngle).
     * Populated by {@link #precompute()} at robot startup; empty until then.
     */
    private static final InterpolatingDoubleTreeMap ANGLE_TABLE = new InterpolatingDoubleTreeMap();

    private ShooterKinematics() {} // Utility class — do not instantiate.

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * Immutable record holding a complete shooter setpoint.
     *
     * @param flywheelRPM  Target flywheel speed in rotations per minute.
     * @param hoodAngleDeg Target hood angle in degrees (mechanism, not motor rotations).
     */
    public record ShooterSetpoint(double flywheelRPM, double hoodAngleDeg) {}

    /**
     * Runs the physics solver across the full shooting range and populates the
     * interpolation tables.  Must be called once from {@code Robot}'s constructor
     * (before any enable) so the tables are ready before the first shot.
     *
     * <p>The solver sweeps hood angles from {@link Shooter#HOOD_MIN_ANGLE_DEG} to
     * {@link Shooter#HOOD_MAX_ANGLE_DEG} at every {@link #PRECOMPUTE_STEP_M} distance
     * increment between {@link SuperstructureConstants#MIN_SHOOT_RANGE_M} and
     * {@link SuperstructureConstants#MAX_SHOOT_RANGE_M}.  Results are written
     * directly into {@link #RPM_TABLE} and {@link #ANGLE_TABLE}.
     *
     * <p>Typical runtime on a roboRIO 2: 50–150 ms — well within the disabled
     * window before driver-station enable.
     */
    public static void precompute() {
        long start = System.currentTimeMillis();
        int  count = 0;

        for (double dist = SuperstructureConstants.MIN_SHOOT_RANGE_M;
             dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M + 1e-9;
             dist += PRECOMPUTE_STEP_M) {

            ShooterSetpoint sp = calculatePhysicsWithDrag(dist);
            RPM_TABLE.put(dist,   sp.flywheelRPM());
            ANGLE_TABLE.put(dist, sp.hoodAngleDeg());
            count++;
        }

        long elapsed = System.currentTimeMillis() - start;
        System.out.printf(
            "[ShooterKinematics] Precomputed %d table entries in %d ms%n", count, elapsed);
    }

    /**
     * Returns the shooter setpoint for the given distance to the HUB center by
     * interpolating the precomputed physics table.
     *
     * <p>{@link #precompute()} must be called before this method.
     *
     * @param distanceToHubMeters Horizontal distance from the robot to the HUB center
     *                            in meters.  Must be positive.
     * @return A {@link ShooterSetpoint} with flywheel RPM and hood angle.
     */
    public static ShooterSetpoint calculate(double distanceToHubMeters) {
        if (distanceToHubMeters <= 0) {
            throw new IllegalArgumentException(
                    "distanceToHubMeters must be positive, got: " + distanceToHubMeters);
        }
        return calculateInterpolated(distanceToHubMeters);
    }

    /**
     * Estimates the ball's flight time from the launcher to the HUB front rim,
     * accounting for aerodynamic drag.  Used by
     * {@link frc.robot.commands.ShootCommand} to compute the moving-while-shooting
     * lead-angle correction.
     *
     * @param distanceToHubMeters Distance to the HUB center in meters.
     * @param setpoint            The setpoint that will be applied (determines launch
     *                            angle and speed).
     * @return Estimated flight time in seconds.
     */
    public static double getFlightTimeSeconds(double distanceToHubMeters,
                                               ShooterSetpoint setpoint) {
        double dRim     = Math.max(0.1, distanceToHubMeters - HEX_RIM_CIRCUMRADIUS_M);
        double v0       = rpmToLaunchSpeed(setpoint.flywheelRPM());
        // Hood angle must be converted to ball exit angle before simulation.
        double thetaRad = Math.toRadians(hoodToBallExitAngleDeg(setpoint.hoodAngleDeg()));
        return simulateFlightTime(v0, thetaRad, dRim);
    }

    // =========================================================================
    // Strategy 1 — 3-Point Parabola with Aerodynamic Drag
    // =========================================================================

    /**
     * Determines the shooter setpoint by fitting a <b>vacuum parabola</b>
     * through three geometric constraint points, then applying drag
     * compensation and tolerance margins.
     *
     * <h3>3-Point Parabola</h3>
     * <p>The trajectory is constrained to pass through:
     * <ol>
     *   <li><b>Shooter exit</b> — ({@code 0}, {@link Shooter#LAUNCH_HEIGHT_M})</li>
     *   <li><b>Front rim clearance</b> — ({@code dRim}, {@code hClearance}),
     *       where {@code hClearance = HUB_TOP_OPENING_HEIGHT_M + FUEL_RADIUS_M
     *       + RIM_SAFETY_MARGIN_M}</li>
     *   <li><b>Hub center</b> — ({@code dCenter},
     *       {@link frc.robot.Constants.Field#HUB_CENTER_HEIGHT_M})</li>
     * </ol>
     * These three points uniquely define a parabola
     * {@code y = h₀ + b·x + a·x²}, from which the ball exit angle
     * {@code θ = atan(b)} is extracted analytically.  No arbitrary drop
     * constant or angle sweep is needed — the geometry alone determines the
     * hood angle.
     *
     * The ball arcs over the front rim at clearance height, then descends
     * through the hub center at the hub's geometric center height (58.125 in,
     * well below the 72 in rim).  Past the center the trajectory continues
     * downward into the basket.  The apex of the arc falls naturally between
     * the rim and the center (it does <em>not</em> need to be perpendicular
     * to the rim).
     *
     * <h3>Drag Compensation</h3>
     * <p>The vacuum parabola sets the <em>angle</em>.  Because aerodynamic
     * drag slows the ball in flight, the actual launch speed must be higher
     * than the vacuum prediction.  A binary search (via
     * {@link #findMinimumLaunchSpeed}) finds the minimum speed at the chosen
     * angle that still clears the front rim in the full drag simulation.
     *
     * <h3>Tolerance Robustness</h3>
     * <ol>
     *   <li><b>Turret yaw error → effective distance:</b> worst-case rim
     *       distance is {@code dRim / cos(TURRET_TOLERANCE_DEG)}.</li>
     *   <li><b>Hood angle error:</b> the rim-clearance speed is evaluated at
     *       nominal and ±{@link Shooter#HOOD_TOLERANCE_DEG} deviations;
     *       the maximum (worst-case) is taken.</li>
     *   <li><b>Flywheel speed error:</b> the velocity deficit from
     *       {@link Shooter#FLYWHEEL_TOLERANCE_RPS} is added so the ball
     *       clears even at the low end of the flywheel tolerance band.</li>
     * </ol>
     */
    private static ShooterSetpoint calculatePhysicsWithDrag(double distanceToHubMeters) {
        double dRimNominal = Math.max(0.1, distanceToHubMeters - HEX_RIM_CIRCUMRADIUS_M);

        // Turret yaw error stretches the horizontal path the ball must travel.
        double dRim = dRimNominal / Math.cos(Math.toRadians(Turret.TURRET_TOLERANCE_DEG));

        // Front rim clearance height (ball center must be at least this high).
        double hClearance = Field.HUB_TOP_OPENING_HEIGHT_M
                          + Field.FUEL_RADIUS_M
                          + Shooter.RIM_SAFETY_MARGIN_M;

        // 3rd constraint: ball arrives at hub center at the hub's geometric
        // center height (58.125 in) — well below the 72 in rim.
        double dCenter = distanceToHubMeters;
        double hCenter = Field.HUB_CENTER_HEIGHT_M;

        // ── Solve vacuum parabola for the launch angle ──────────────────
        double hoodDeg = solveVacuumHoodAngle(dRim, hClearance, dCenter, hCenter);

        // Clamp to physical mechanism limits.
        hoodDeg = Math.max(Shooter.HOOD_MIN_ANGLE_DEG,
                  Math.min(Shooter.HOOD_MAX_ANGLE_DEG, hoodDeg));

        // ── Tolerance-robust launch speed ────────────────────────────────
        double hoodLo = Math.max(Shooter.HOOD_MIN_ANGLE_DEG,
                                 hoodDeg - Shooter.HOOD_TOLERANCE_DEG);
        double hoodHi = Math.min(Shooter.HOOD_MAX_ANGLE_DEG,
                                 hoodDeg + Shooter.HOOD_TOLERANCE_DEG);

        double vToleranceMps = rpmToLaunchSpeed(Shooter.FLYWHEEL_TOLERANCE_RPS * 60.0);

        // Find the minimum launch speed that clears the front rim at the
        // worst-case hood deviation (nominal, +tol, −tol).
        double worstV0 = 0.0;
        for (double h : new double[]{hoodLo, hoodDeg, hoodHi}) {
            double thetaRad = Math.toRadians(hoodToBallExitAngleDeg(h));
            double v0       = findMinimumLaunchSpeed(thetaRad, dRim, hClearance);
            worstV0 = Math.max(worstV0, v0);
        }

        // Add flywheel tolerance so the ball clears even if the flywheel
        // is spinning at (commanded − FLYWHEEL_TOLERANCE_RPS).
        double robustV0 = worstV0 + vToleranceMps;

        if (robustV0 >= MAX_LAUNCH_SPEED_MPS) {
            return calculateInterpolated(distanceToHubMeters);
        }

        return new ShooterSetpoint(v0ToRPM(robustV0), hoodDeg);
    }

    /**
     * Solves for the hood angle that produces a vacuum parabola through three
     * constraint points:
     * <ol>
     *   <li>Shooter exit:  {@code (0, LAUNCH_HEIGHT_M)}</li>
     *   <li>Rim clearance: {@code (dRim, hRim)}</li>
     *   <li>Hub center:    {@code (dCenter, HUB_CENTER_HEIGHT_M)}</li>
     * </ol>
     *
     * <p>The vacuum trajectory is {@code y = h₀ + b·x + a·x²} where
     * {@code b = tan(θ)} (launch slope) and
     * {@code a = −g / (2·v₀²·cos²θ)} (gravity curvature).  Substituting
     * the two non-origin points gives a {@code 2×2} linear system in
     * {@code (a, b)} with determinant
     * {@code det = dRim · dCenter · (dRim − dCenter)}, which is always
     * non-zero because {@code dRim < dCenter}.
     *
     * @param dRim    Horizontal distance to front rim (m).
     * @param hRim    Required clearance height at the rim (m).
     * @param dCenter Horizontal distance to hub center (m).
     * @param hCenter Target height at hub center (m).
     * @return Hood mechanism angle in degrees ({@code 90° − θ}).
     *         May be outside physical limits; caller must clamp.
     */
    private static double solveVacuumHoodAngle(double dRim, double hRim,
                                               double dCenter, double hCenter) {
        double h0 = Shooter.LAUNCH_HEIGHT_M;
        double d1 = hRim    - h0; // height gain to rim
        double d2 = hCenter - h0; // height gain to center

        double det = dRim * dCenter * (dRim - dCenter);

        // Parabola: y = h0 + b·x + a·x²
        // Only b (= tan θ) is needed to determine the launch angle.
        double b = (dRim * dRim * d2 - dCenter * dCenter * d1) / det;

        double thetaDeg = Math.toDegrees(Math.atan(b));
        return 90.0 - thetaDeg; // hood angle = 90° − ball exit angle
    }

    /**
     * Binary-searches for the minimum launch speed (m/s) at the given angle that
     * places the ball at or above {@code targetHeightM} when it reaches {@code targetXM}.
     *
     * @return Minimum launch speed in m/s, or {@link #MAX_LAUNCH_SPEED_MPS} if no
     *         valid solution exists for this angle.
     */
    private static double findMinimumLaunchSpeed(double thetaRad,
                                                  double targetXM,
                                                  double targetHeightM) {
        // Check that max speed can even reach the target height.
        if (simulateHeightAtX(MAX_LAUNCH_SPEED_MPS, thetaRad, targetXM) < targetHeightM) {
            return MAX_LAUNCH_SPEED_MPS; // No valid solution for this angle.
        }

        double lo = 0.0, hi = MAX_LAUNCH_SPEED_MPS;
        for (int i = 0; i < BINARY_SEARCH_ITERS; i++) {
            double mid = (lo + hi) / 2.0;
            if (simulateHeightAtX(mid, thetaRad, targetXM) >= targetHeightM) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        return hi;
    }

    /**
     * Simulates projectile motion with aerodynamic drag and returns the ball's
     * height (meters) when it reaches horizontal position {@code targetXM}.
     * Returns {@code Double.NEGATIVE_INFINITY} if the ball hits the ground or
     * stalls before reaching the target.
     *
     * <p>When the final Euler step overshoots {@code targetXM}, the returned
     * height is linearly interpolated back to the exact crossing point.  This
     * prevents an optimistic bias of up to {@code tan(θ) × vx × dt} that would
     * otherwise cause the solver to accept shots that clip the rim.
     */
    private static double simulateHeightAtX(double v0, double thetaRad, double targetXM) {
        double vx = v0 * Math.cos(thetaRad);
        double vy = v0 * Math.sin(thetaRad);
        double x  = 0.0;
        double y  = Shooter.LAUNCH_HEIGHT_M;

        for (int step = 0; step < MAX_SIM_STEPS; step++) {
            if (y < 0.0 || vx <= 0.0) {
                return Double.NEGATIVE_INFINITY;
            }
            double xPrev = x;
            double yPrev = y;

            double v  = Math.sqrt(vx * vx + vy * vy);
            double ax = -DRAG_ACCEL_COEFF * v * vx;
            double ay = -GRAVITY_MPS2 - DRAG_ACCEL_COEFF * v * vy;
            // Update position BEFORE velocity (forward Euler) so x always advances
            // with the speed from the start of this step, preventing negative-vx
            // from reversing x on the same step it is detected.
            x  += vx * SIM_DT_S;
            y  += vy * SIM_DT_S;
            vx += ax * SIM_DT_S;
            vy += ay * SIM_DT_S;

            // Interpolate to exact target-x crossing to eliminate forward-Euler
            // overshoot bias (up to tan(θ) × vx × dt ≈ 5-10 cm for ascending
            // trajectories — far more than RIM_SAFETY_MARGIN_M).
            if (x >= targetXM) {
                double frac = (targetXM - xPrev) / (x - xPrev);
                return yPrev + frac * (y - yPrev);
            }
        }
        return y;
    }

    /**
     * Simulates trajectory with drag and returns the elapsed time when the ball
     * reaches horizontal position {@code targetXM}.  Uses the same endpoint
     * interpolation as {@link #simulateHeightAtX} for sub-step accuracy.
     */
    private static double simulateFlightTime(double v0, double thetaRad, double targetXM) {
        double vx = v0 * Math.cos(thetaRad);
        double vy = v0 * Math.sin(thetaRad);
        double x  = 0.0;
        double y  = Shooter.LAUNCH_HEIGHT_M;
        double t  = 0.0;

        for (int step = 0; step < MAX_SIM_STEPS; step++) {
            if (y < 0.0 || vx <= 0.0) break;

            double xPrev = x;

            double v  = Math.sqrt(vx * vx + vy * vy);
            double ax = -DRAG_ACCEL_COEFF * v * vx;
            double ay = -GRAVITY_MPS2 - DRAG_ACCEL_COEFF * v * vy;
            // Advance position first (forward Euler) so this step's x increment
            // uses the current speed, not a potentially-stalled post-update speed.
            x  += vx * SIM_DT_S;
            y  += vy * SIM_DT_S;
            vx += ax * SIM_DT_S;
            vy += ay * SIM_DT_S;
            t  += SIM_DT_S;

            if (x >= targetXM) {
                double frac = (targetXM - xPrev) / (x - xPrev);
                return (t - SIM_DT_S) + frac * SIM_DT_S;
            }
        }
        return t;
    }

    // =========================================================================
    // Strategy 2 — Interpolation Table
    // =========================================================================

    /**
     * Returns a setpoint by interpolating the empirical lookup tables.
     * The {@link InterpolatingDoubleTreeMap} clamps to the nearest bound when
     * queried outside the populated range.
     */
    private static ShooterSetpoint calculateInterpolated(double distanceToHubMeters) {
        double rpm      = RPM_TABLE.get(distanceToHubMeters);
        double angleDeg = ANGLE_TABLE.get(distanceToHubMeters);
        angleDeg = Math.max(Shooter.HOOD_MIN_ANGLE_DEG,
                   Math.min(Shooter.HOOD_MAX_ANGLE_DEG, angleDeg));
        return new ShooterSetpoint(rpm, angleDeg);
    }

    // =========================================================================
    // Hood / Launch Angle Conversion
    // =========================================================================

    /**
     * Converts a hood mechanism angle to the ball's actual exit (launch) angle.
     *
     * <p>For this flywheel shooter the hood deflects the ball so the relationship
     * is inverted: {@code ballExitAngle = 90° − hoodAngle}.
     * Hood at 90° → ball exits flat (0°); hood at 20° → ball exits at 70° (steep).
     *
     * @param hoodAngleDeg Hood mechanism angle in degrees.
     * @return Ball exit angle in degrees, measured from horizontal.
     */
    private static double hoodToBallExitAngleDeg(double hoodAngleDeg) {
        return 90.0 - hoodAngleDeg;
    }

    // =========================================================================
    // Unit Conversion Helpers
    // =========================================================================

    /**
     * Converts motor RPM to estimated ball launch speed in m/s.
     * <pre>
     *   v_surface = (RPM / FLYWHEEL_GEAR_RATIO) × 2π × FLYWHEEL_WHEEL_RADIUS_M / 60
     *   v_ball    = v_surface × FLYWHEEL_EFFICIENCY
     * </pre>
     * {@link Shooter#FLYWHEEL_GEAR_RATIO} is the motor-to-bottom-flywheel reduction.
     * The top (hood) wheels are geared to the same tangential surface speed, so
     * either wheel set gives the same {@code v_ball}.
     */
    private static double rpmToLaunchSpeed(double rpm) {
        return rpm / Shooter.FLYWHEEL_GEAR_RATIO
               * 2.0 * Math.PI * Shooter.FLYWHEEL_WHEEL_RADIUS_M
               * Shooter.FLYWHEEL_EFFICIENCY / 60.0;
    }

    /**
     * Converts ball launch speed in m/s to the required motor RPM.
     * <pre>
     *   RPM = v_ball × 60 × FLYWHEEL_GEAR_RATIO / (2π × FLYWHEEL_WHEEL_RADIUS_M × FLYWHEEL_EFFICIENCY)
     * </pre>
     */
    private static double v0ToRPM(double v0) {
        return v0 * 60.0 * Shooter.FLYWHEEL_GEAR_RATIO
               / (2.0 * Math.PI * Shooter.FLYWHEEL_WHEEL_RADIUS_M * Shooter.FLYWHEEL_EFFICIENCY);
    }
}
