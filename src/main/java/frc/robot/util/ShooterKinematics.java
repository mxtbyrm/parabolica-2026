package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.robot.Constants.Field;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.SuperstructureConstants;

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
 * For each candidate hood angle (swept from {@link Shooter#HOOD_MIN_ANGLE_DEG}
 * to {@link Shooter#HOOD_MAX_ANGLE_DEG} in 1° increments) the corresponding ball
 * exit angle is computed and a binary search finds the minimum launch speed that
 * clears the hub rim with the required safety margin.
 * The hood angle with the lowest resulting RPM is selected.
 *
 * <p>Results are cached by distance; the solver only runs when the reported
 * distance changes by more than {@link #CACHE_THRESHOLD_M}.
 *
 * <h2>Interpolation Table (empirical fallback)</h2>
 * <p>When {@link #PREFER_PHYSICS_MODEL} is {@code false}, a pre-populated
 * {@link InterpolatingDoubleTreeMap} returns tuned setpoints.  Switch to
 * {@code false} after real-robot characterization sessions.
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

    /** Angle sweep resolution in degrees. */
    private static final double ANGLE_SWEEP_STEP_DEG = 1.0;

    /** Binary search iteration count for minimum launch speed. */
    private static final int BINARY_SEARCH_ITERS = 35;

    /**
     * Maximum Euler steps per simulation call (= 50 s at SIM_DT_S = 10 ms).
     * Prevents infinite loops if drag causes vx to approach zero asymptotically.
     */
    private static final int MAX_SIM_STEPS = 5000;

    /** Maximum plausible launch speed in m/s for binary search upper bound. */
    private static final double MAX_LAUNCH_SPEED_MPS = 30.0;

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
        double dRim     = Math.max(0.1, distanceToHubMeters - Field.HUB_TOP_OPENING_DIAMETER_M / 2.0);
        double v0       = rpmToLaunchSpeed(setpoint.flywheelRPM());
        // Hood angle must be converted to ball exit angle before simulation.
        double thetaRad = Math.toRadians(hoodToBallExitAngleDeg(setpoint.hoodAngleDeg()));
        return simulateFlightTime(v0, thetaRad, dRim);
    }

    // =========================================================================
    // Strategy 1 — Physics Model with Aerodynamic Drag
    // =========================================================================

    /**
     * Sweeps launch angles and binary-searches for the minimum-energy setpoint
     * that clears the HUB rim with the configured safety margin.  Trajectory
     * simulation includes aerodynamic drag via Euler integration.
     */
    private static ShooterSetpoint calculatePhysicsWithDrag(double distanceToHubMeters) {
        double dRim       = Math.max(0.1, distanceToHubMeters - Field.HUB_TOP_OPENING_DIAMETER_M / 2.0);
        double hClearance = Field.HUB_TOP_OPENING_HEIGHT_M
                          + Field.FUEL_RADIUS_M
                          + Shooter.RIM_SAFETY_MARGIN_M;

        double bestRPM    = Double.MAX_VALUE;
        double bestAngle  = Shooter.HOOD_MIN_ANGLE_DEG;

        for (double hoodDeg = Shooter.HOOD_MIN_ANGLE_DEG;
             hoodDeg <= Shooter.HOOD_MAX_ANGLE_DEG;
             hoodDeg += ANGLE_SWEEP_STEP_DEG) {

            // Convert hood angle to actual ball exit angle before simulating.
            double thetaRad = Math.toRadians(hoodToBallExitAngleDeg(hoodDeg));
            double v0       = findMinimumLaunchSpeed(thetaRad, dRim, hClearance);

            if (v0 < MAX_LAUNCH_SPEED_MPS) { // valid solution found
                double rpm = v0ToRPM(v0);
                if (rpm < bestRPM) {
                    bestRPM   = rpm;
                    bestAngle = hoodDeg;
                }
            }
        }

        // If no solution found (extremely short or long range), fall back to table.
        if (bestRPM == Double.MAX_VALUE) {
            return calculateInterpolated(distanceToHubMeters);
        }

        return new ShooterSetpoint(bestRPM, bestAngle);
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
     */
    private static double simulateHeightAtX(double v0, double thetaRad, double targetXM) {
        double vx = v0 * Math.cos(thetaRad);
        double vy = v0 * Math.sin(thetaRad);
        double x  = 0.0;
        double y  = Shooter.LAUNCH_HEIGHT_M;

        for (int step = 0; step < MAX_SIM_STEPS && x < targetXM; step++) {
            if (y < 0.0 || vx <= 0.0) {
                return Double.NEGATIVE_INFINITY;
            }
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
        }
        return y;
    }

    /**
     * Simulates trajectory with drag and returns the elapsed time when the ball
     * reaches horizontal position {@code targetXM}.
     */
    private static double simulateFlightTime(double v0, double thetaRad, double targetXM) {
        double vx = v0 * Math.cos(thetaRad);
        double vy = v0 * Math.sin(thetaRad);
        double x  = 0.0;
        double y  = Shooter.LAUNCH_HEIGHT_M;
        double t  = 0.0;

        for (int step = 0; step < MAX_SIM_STEPS && x < targetXM && y > 0.0 && vx > 0.0; step++) {
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
