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
 * <h2>Drag-Aware 3-Point Solver</h2>
 * <p>The trajectory must satisfy two constraints simultaneously:
 * <ol>
 *   <li><b>Front rim clearance</b> — ball height at {@code dRim} ≥
 *       {@code HUB_TOP_OPENING_HEIGHT_M + FUEL_RADIUS_M + RIM_SAFETY_MARGIN_M}</li>
 *   <li><b>Hub center targeting</b> — ball height at {@code dCenter} =
 *       {@link frc.robot.Constants.Field#HUB_CENTER_HEIGHT_M} (58.125 in)</li>
 * </ol>
 * Both constraints are evaluated using full Euler-integrated drag simulation.
 *
 * <p>A <b>vacuum parabola</b> through the three geometric points (shooter exit,
 * rim clearance, hub center) provides the initial angle estimate.  A nested
 * binary search then refines both the launch angle and speed with drag:
 * <ul>
 *   <li><b>Inner search (speed):</b> for a given angle, find the minimum v₀
 *       that clears the front rim in the drag simulation.</li>
 *   <li><b>Outer search (angle):</b> find the angle where, at the
 *       rim-clearing speed, the ball arrives at hub center at exactly
 *       {@code HUB_CENTER_HEIGHT_M}.</li>
 * </ul>
 * This fully accounts for drag on both the angle and speed — no vacuum
 * approximations remain in the final setpoint.
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

    /**
     * Validity table.  Keys = distance to hub in meters; values = 1.0 (valid)
     * or 0.0 (invalid).  Populated alongside RPM_TABLE / ANGLE_TABLE during
     * {@link #precompute()}.  {@link #isShootable(double)} looks up this table
     * at runtime — no simulation is run during a match.
     */
    private static final InterpolatingDoubleTreeMap VALID_TABLE = new InterpolatingDoubleTreeMap();

    /**
     * Pass RPM table.  Keys = horizontal distance to pass target (m);
     * values = flywheel RPM needed to deliver the ball to
     * {@link Shooter#PASS_TARGET_HEIGHT_M} at that distance using the
     * minimum-energy hood angle.  Populated by {@link #precompute()}.
     */
    private static final InterpolatingDoubleTreeMap PASS_RPM_TABLE   = new InterpolatingDoubleTreeMap();

    /**
     * Pass hood-angle table.  Keys = horizontal distance to pass target (m);
     * values = hood angle in degrees.  Populated by {@link #precompute()}.
     */
    private static final InterpolatingDoubleTreeMap PASS_ANGLE_TABLE = new InterpolatingDoubleTreeMap();

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

        int validCount = 0;
        for (double dist = SuperstructureConstants.MIN_SHOOT_RANGE_M;
             dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M + 1e-9;
             dist += PRECOMPUTE_STEP_M) {

            ShooterSetpoint sp = calculatePhysicsWithDrag(dist);
            RPM_TABLE.put(dist,   sp.flywheelRPM());
            ANGLE_TABLE.put(dist, sp.hoodAngleDeg());

            // Validate the computed setpoint once here at init — never again at runtime.
            boolean valid = checkTrajectoryConstraints(dist, sp);
            VALID_TABLE.put(dist, valid ? 1.0 : 0.0);
            if (valid) validCount++;
            count++;
        }

        // Pass table — no rim constraint, target is PASS_TARGET_HEIGHT_M.
        // Uses MAX_PASS_RANGE_M (larger than MAX_SHOOT_RANGE_M) so long
        // cross-field passes from the far side of the neutral zone get the
        // correct RPM instead of being clamped to the 7 m shooting range.
        for (double dist = SuperstructureConstants.MIN_SHOOT_RANGE_M;
             dist <= SuperstructureConstants.MAX_PASS_RANGE_M + 1e-9;
             dist += PRECOMPUTE_STEP_M) {
            ShooterSetpoint sp = calculatePassPhysics(dist);
            PASS_RPM_TABLE.put(dist,   sp.flywheelRPM());
            PASS_ANGLE_TABLE.put(dist, sp.hoodAngleDeg());
        }

        long elapsed = System.currentTimeMillis() - start;
        System.out.printf(
            "[ShooterKinematics] Precomputed %d table entries (%d valid) in %d ms%n",
            count, validCount, elapsed);

        // Print the full table to stdout at startup.
        // Visible in the Driver Station console / RioLog after deploy.
        // Check: are all entries valid (✓)? Invalid entries = robot won't shoot at that distance.
        printTable();
    }

    /** Nominal battery voltage the precomputed tables assume.  Flywheel
     *  efficiency and PIDF gains are calibrated at this voltage. */
    private static final double NOMINAL_VOLTAGE = 12.0;

    /** Minimum voltage for compensation.  Below this the robot has bigger
     *  problems than shot accuracy (brownout territory). */
    private static final double MIN_COMPENSATION_VOLTAGE = 8.0;

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
        // Clamp to the valid shooting range instead of throwing — a bad vision
        // reading should not crash the robot mid-match.
        distanceToHubMeters = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                              Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M,
                                       distanceToHubMeters));
        return calculateInterpolated(distanceToHubMeters);
    }

    /**
     * Returns the shooter setpoint with battery-voltage compensation.
     *
     * <p>The precomputed table assumes {@link #NOMINAL_VOLTAGE} (12 V).  At lower
     * battery voltage the flywheel motor produces less torque, so the ball exits
     * slower than the commanded RPM would suggest.  This overload bumps the
     * flywheel RPM by the ratio {@code NOMINAL_VOLTAGE / actualVoltage} so that
     * the ball still achieves the physics-model-predicted launch speed.
     *
     * <p>The hood angle is <em>not</em> compensated — the hood uses MotionMagic
     * position control which tracks position accurately regardless of bus voltage
     * (it just moves slower at low voltage, not to a different position).
     *
     * @param distanceToHubMeters Horizontal distance to HUB center in meters.
     *                            Clamped to [{@link SuperstructureConstants#MIN_SHOOT_RANGE_M},
     *                            {@link SuperstructureConstants#MAX_SHOOT_RANGE_M}].
     * @param batteryVoltage      Current battery voltage from
     *                            {@code RobotController.getBatteryVoltage()}.
     * @return A {@link ShooterSetpoint} with voltage-compensated RPM and hood angle.
     */
    public static ShooterSetpoint calculate(double distanceToHubMeters,
                                             double batteryVoltage) {
        ShooterSetpoint base = calculate(distanceToHubMeters);
        double clampedVoltage = Math.max(MIN_COMPENSATION_VOLTAGE,
                                Math.min(NOMINAL_VOLTAGE, batteryVoltage));
        double voltageScale = NOMINAL_VOLTAGE / clampedVoltage;
        // RPMScale is already applied inside calculateInterpolated() (via the base
        // calculate() call above) — do NOT apply it again here or it would double.
        return new ShooterSetpoint(base.flywheelRPM() * voltageScale, base.hoodAngleDeg());
    }

    /**
     * Returns the shooter setpoint for a <b>pass</b> to an alliance partner at
     * {@code distanceM}.  Unlike {@link #calculate(double)}, this solver has no
     * hub rim to clear: it finds the minimum-RPM (flattest feasible) trajectory
     * that delivers the ball to horizontal distance {@code distanceM} at
     * {@link Shooter#PASS_TARGET_HEIGHT_M} above the floor.
     *
     * <p>{@link #precompute()} must be called before this method.
     *
     * @param distanceM Horizontal distance from the turret pivot to the pass
     *                  target in meters.  Clamped to the valid shooting range.
     * @return A {@link ShooterSetpoint} tuned for flat pass delivery.
     */
    public static ShooterSetpoint calculatePass(double distanceM) {
        distanceM = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                    Math.min(SuperstructureConstants.MAX_PASS_RANGE_M, distanceM));
        double rpm  = PASS_RPM_TABLE.get(distanceM);
        double hood = PASS_ANGLE_TABLE.get(distanceM);
        hood = Math.max(Shooter.HOOD_MIN_ANGLE_DEG,
               Math.min(Shooter.HOOD_MAX_ANGLE_DEG, hood));
        return new ShooterSetpoint(rpm, hood);
    }

    /**
     * Returns {@code true} if the robot can make a physically valid shot from
     * {@code distanceToHubMeters}.  O(1) table lookup — the simulation ran
     * once during {@link #precompute()} at robot init; no physics is executed
     * at runtime.
     *
     * <p>Returns {@code false} if the distance is outside
     * [{@link SuperstructureConstants#MIN_SHOOT_RANGE_M},
     * {@link SuperstructureConstants#MAX_SHOOT_RANGE_M}] or if the precomputed
     * trajectory for this distance fails the rim-clearance or hub-center constraint.
     *
     * @param distanceToHubMeters Actual robot-to-hub distance in meters (not clamped).
     * @return {@code true} if the distance is shootable.
     */
    public static boolean isShootable(double distanceToHubMeters) {
        if (distanceToHubMeters < SuperstructureConstants.MIN_SHOOT_RANGE_M
                || distanceToHubMeters > SuperstructureConstants.MAX_SHOOT_RANGE_M) {
            return false;
        }
        return VALID_TABLE.get(distanceToHubMeters) >= 0.5;
    }

    /**
     * Runs the drag-aware Euler simulation to verify both trajectory constraints
     * for a precomputed setpoint.  Called once per distance entry during
     * {@link #precompute()} — never during a match.
     *
     * <p>Two constraints are checked:
     * <ol>
     *   <li><b>Near-rim clearance:</b> ball center is at or above
     *       {@code HUB_TOP_OPENING_HEIGHT_M + FUEL_RADIUS_M + RIM_SAFETY_MARGIN_M}
     *       when it crosses the near rim.</li>
     *   <li><b>Hub entry:</b> ball center has descended back below the hub opening
     *       level ({@code HUB_TOP_OPENING_HEIGHT_M + FUEL_RADIUS_M}) by the time
     *       it reaches the far rim ({@code distanceM + HEX_RIM_CIRCUMRADIUS_M}).
     *       This verifies the ball actually arcs INTO the hub rather than flying over.</li>
     * </ol>
     */
    private static boolean checkTrajectoryConstraints(double distanceM,
                                                       ShooterSetpoint setpoint) {
        double v0       = rpmToLaunchSpeed(setpoint.flywheelRPM());
        double thetaRad = Math.toRadians(hoodToBallExitAngleDeg(setpoint.hoodAngleDeg()));

        // Constraint 1: clear the near rim.
        double dNear     = Math.max(0.1, distanceM - HEX_RIM_CIRCUMRADIUS_M);
        double hClearance = Field.HUB_TOP_OPENING_HEIGHT_M
                          + Field.FUEL_RADIUS_M
                          + Shooter.RIM_SAFETY_MARGIN_M;
        if (simulateHeightAtX(v0, thetaRad, dNear) < hClearance) return false;

        // Constraint 2: enter the hub (don't clip the far rim).
        // Ball center must be below rim + ball-radius at the far rim, meaning it
        // has actually entered the hub opening rather than flying over it.
        double dFar        = distanceM + HEX_RIM_CIRCUMRADIUS_M;
        double hFar        = simulateHeightAtX(v0, thetaRad, dFar);
        double hRimPlusBall = Field.HUB_TOP_OPENING_HEIGHT_M + Field.FUEL_RADIUS_M;
        return hFar < hRimPlusBall;
    }

    /**
     * Returns the ball's horizontal ground-frame speed component for the given
     * setpoint.  Used by the SOTM lead-angle formula:
     * <pre>
     *   leadAngle = asin(-vLateral / getHorizontalSpeedMps(setpoint))
     * </pre>
     * This is the exact solution for the horizontal-plane lead angle: to zero out
     * the net lateral displacement during flight, the turret must aim so that
     * {@code v_h * sin(leadAngle) + vLateral = 0}.
     *
     * @param setpoint The shooter setpoint (RPM + hood angle).
     * @return Horizontal launch speed in m/s.
     */
    public static double getHorizontalSpeedMps(ShooterSetpoint setpoint) {
        double v0          = rpmToLaunchSpeed(setpoint.flywheelRPM());
        double exitAngleDeg = hoodToBallExitAngleDeg(setpoint.hoodAngleDeg());
        return v0 * Math.cos(Math.toRadians(exitAngleDeg));
    }

    /**
     * Estimates the ball's flight time from the launcher to the HUB center,
     * accounting for aerodynamic drag.  Used by
     * {@link frc.robot.commands.ShootCommand} to compute the moving-while-shooting
     * position prediction horizon.
     *
     * @param distanceToHubMeters Distance to the HUB center in meters.
     * @param setpoint            The setpoint that will be applied (determines launch
     *                            angle and speed).
     * @return Estimated flight time in seconds.
     */
    public static double getFlightTimeSeconds(double distanceToHubMeters,
                                               ShooterSetpoint setpoint) {
        double v0       = rpmToLaunchSpeed(setpoint.flywheelRPM());
        double thetaRad = Math.toRadians(hoodToBallExitAngleDeg(setpoint.hoodAngleDeg()));
        double vx0      = v0 * Math.cos(thetaRad);
        double vy0      = v0 * Math.sin(thetaRad);
        return simulateFlightTimeComponents(vx0, vy0, distanceToHubMeters);
    }


    // =========================================================================
    // Pass Solver — minimum-RPM flat delivery to PASS_TARGET_HEIGHT_M
    // =========================================================================

    /**
     * Finds the minimum-RPM (hood angle, launch speed) pair that delivers the
     * ball to {@code (distanceM, PASS_TARGET_HEIGHT_M)} on a descending arc.
     *
     * <p>Algorithm: sweep hood angles from flattest ({@link Shooter#HOOD_MAX_ANGLE_DEG})
     * to steepest ({@link Shooter#HOOD_MIN_ANGLE_DEG}).  For each angle, binary-search
     * for the launch speed that places the ball at {@code PASS_TARGET_HEIGHT_M} when
     * {@code x = distanceM}.  Return the combination with the lowest RPM
     * (flattest feasible trajectory = easiest for partner intake).
     */
    private static ShooterSetpoint calculatePassPhysics(double distanceM) {
        double targetHeight = Shooter.PASS_TARGET_HEIGHT_M;
        double bestRPM  = Double.MAX_VALUE;
        double bestHood = Shooter.HOOD_MAX_ANGLE_DEG;

        // Sweep from flattest (max hood) to steepest (min hood) in 0.5° steps.
        for (double hood = Shooter.HOOD_MAX_ANGLE_DEG;
             hood >= Shooter.HOOD_MIN_ANGLE_DEG - 1e-9;
             hood -= 0.5) {
            double thetaRad = Math.toRadians(hoodToBallExitAngleDeg(hood));
            double v0 = findSpeedForTargetHeight(thetaRad, distanceM, targetHeight);
            if (v0 >= MAX_LAUNCH_SPEED_MPS) continue; // no solution at this angle
            double rpm = v0ToRPM(v0);
            if (rpm < bestRPM) {
                bestRPM  = rpm;
                bestHood = hood;
            }
        }

        if (bestRPM == Double.MAX_VALUE) {
            // No valid solution in range — return flattest max-power attempt.
            double thetaRad = Math.toRadians(hoodToBallExitAngleDeg(Shooter.HOOD_MAX_ANGLE_DEG));
            double v0 = findSpeedForTargetHeight(
                    thetaRad, distanceM, Shooter.LAUNCH_HEIGHT_M - 0.01);
            bestRPM  = v0 < MAX_LAUNCH_SPEED_MPS ? v0ToRPM(v0) : v0ToRPM(MAX_LAUNCH_SPEED_MPS * 0.9);
            bestHood = Shooter.HOOD_MAX_ANGLE_DEG;
        }

        return new ShooterSetpoint(bestRPM, bestHood);
    }

    /**
     * Binary-searches for the launch speed (m/s) at angle {@code thetaRad} that
     * places the ball at exactly {@code targetHeight} when {@code x = targetXM}.
     *
     * <p>Height at x is monotonically increasing with speed for practical pass
     * trajectories (more speed → ball arrives earlier in arc / with more energy
     * → higher at the target plane).
     *
     * @return Launch speed in m/s, or {@link #MAX_LAUNCH_SPEED_MPS} if no solution.
     */
    private static double findSpeedForTargetHeight(double thetaRad,
                                                    double targetXM,
                                                    double targetHeight) {
        // If even max speed can't reach the target height, no solution.
        if (simulateHeightAtX(MAX_LAUNCH_SPEED_MPS, thetaRad, targetXM) < targetHeight) {
            return MAX_LAUNCH_SPEED_MPS;
        }
        // If minimum speed (near-zero) already exceeds target height, no solution
        // in a useful range — the ball is always too high at this angle.
        if (simulateHeightAtX(0.5, thetaRad, targetXM) > targetHeight) {
            return MAX_LAUNCH_SPEED_MPS;
        }
        double lo = 0.5, hi = MAX_LAUNCH_SPEED_MPS;
        for (int i = 0; i < BINARY_SEARCH_ITERS; i++) {
            double mid = (lo + hi) / 2.0;
            if (simulateHeightAtX(mid, thetaRad, targetXM) < targetHeight) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return hi;
    }

    // =========================================================================
    // Strategy 1 — Drag-Aware 3-Point Solver
    // =========================================================================

    /** Angle binary search iteration count (30 → precision ≈ 0.00005°). */
    private static final int ANGLE_SEARCH_ITERS = 30;

    /**
     * Determines the shooter setpoint using a <b>fully drag-aware</b> nested
     * binary search.
     *
     * <h3>Two Constraints</h3>
     * <ol>
     *   <li><b>Front rim clearance:</b> ball height at {@code dRim} ≥ {@code hClearance}</li>
     *   <li><b>Hub center targeting:</b> ball height at {@code dCenter} = {@code HUB_CENTER_HEIGHT_M}</li>
     * </ol>
     *
     * <h3>Algorithm</h3>
     * <ol>
     *   <li>Compute a vacuum-parabola angle as the initial guess.</li>
     *   <li><b>Outer binary search on hood angle:</b> for each candidate angle,
     *       the inner search finds the minimum launch speed that clears the
     *       front rim (with drag).  Then the full drag simulation checks the
     *       ball height at hub center.  The outer search converges on the
     *       angle where that height equals {@code HUB_CENTER_HEIGHT_M}.</li>
     *   <li>Apply tolerance margins (hood error, flywheel error, turret yaw
     *       error) to the converged setpoint.</li>
     * </ol>
     *
     * <h3>Tolerance Robustness</h3>
     * <ol>
     *   <li><b>Turret yaw error → effective distance:</b> worst-case rim
     *       distance is {@code dRim / cos(TURRET_TOLERANCE_DEG)}.</li>
     *   <li><b>Hood angle error:</b> rim-clearance speed evaluated at
     *       nominal and ±{@link Shooter#HOOD_TOLERANCE_DEG}; maximum taken.</li>
     *   <li><b>Flywheel speed error:</b> velocity deficit from
     *       {@link Shooter#FLYWHEEL_TOLERANCE_RPS} added so the ball clears
     *       at the low end of the flywheel tolerance band.</li>
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

        // Hub center targeting.
        double dCenter = distanceToHubMeters;
        double hCenter = Field.HUB_CENTER_HEIGHT_M;

        // ── Step 1: vacuum parabola for initial angle guess ─────────────
        double vacuumHoodDeg = solveVacuumHoodAngle(dRim, hClearance, dCenter, hCenter);

        // ── Step 2: outer binary search on hood angle (drag-aware) ──────
        // Search range: vacuum angle ± 10° (drag shifts the optimal angle
        // only a few degrees; 10° is generous).
        double angleLo = Math.max(Shooter.HOOD_MIN_ANGLE_DEG, vacuumHoodDeg - 10.0);
        double angleHi = Math.min(Shooter.HOOD_MAX_ANGLE_DEG, vacuumHoodDeg + 10.0);

        // For the binary search to work, we need the height-at-center to be
        // monotonic in hood angle.  Steeper hood angle (higher value) → steeper
        // launch → ball arrives higher at center.  So:
        //   heightAtCenter(angleLo) should be < hCenter  (too flat, ball drops too much)
        //   heightAtCenter(angleHi) should be > hCenter  (too steep, ball still high)
        // If this doesn't hold, we fall back to the vacuum angle.

        double hoodDeg = vacuumHoodDeg; // default to vacuum guess

        // Verify the search bracket is valid.
        double hAtLo = heightAtCenterForHood(angleLo, dRim, hClearance, dCenter);
        double hAtHi = heightAtCenterForHood(angleHi, dRim, hClearance, dCenter);

        if (hAtLo <= hCenter && hAtHi >= hCenter) {
            // Valid bracket — binary search for the exact angle.
            for (int i = 0; i < ANGLE_SEARCH_ITERS; i++) {
                double mid = (angleLo + angleHi) / 2.0;
                double hMid = heightAtCenterForHood(mid, dRim, hClearance, dCenter);
                if (hMid < hCenter) {
                    angleLo = mid; // need steeper angle (more loft)
                } else {
                    angleHi = mid; // can go flatter
                }
            }
            hoodDeg = (angleLo + angleHi) / 2.0;
        }

        // Clamp to physical mechanism limits.
        hoodDeg = Math.max(Shooter.HOOD_MIN_ANGLE_DEG,
                  Math.min(Shooter.HOOD_MAX_ANGLE_DEG, hoodDeg));

        // ── Step 3: tolerance-robust launch speed ────────────────────────
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
            // Distance is outside achievable range — return a sentinel setpoint.
            // checkTrajectoryConstraints() will mark this entry invalid in VALID_TABLE.
            return new ShooterSetpoint(0.0, Shooter.HOOD_MIN_ANGLE_DEG);
        }

        return new ShooterSetpoint(v0ToRPM(robustV0), hoodDeg);
    }

    /**
     * Helper for the outer binary search: given a hood angle, finds the
     * rim-clearing launch speed (with drag) and returns the simulated ball
     * height at hub center.
     *
     * @return Ball height at {@code dCenter} in meters, or
     *         {@link Double#NEGATIVE_INFINITY} if no valid speed exists.
     */
    private static double heightAtCenterForHood(double hoodDeg,
                                                 double dRim, double hClearance,
                                                 double dCenter) {
        double thetaRad = Math.toRadians(hoodToBallExitAngleDeg(hoodDeg));
        double v0 = findMinimumLaunchSpeed(thetaRad, dRim, hClearance);
        if (v0 >= MAX_LAUNCH_SPEED_MPS) return Double.NEGATIVE_INFINITY;
        return simulateHeightAtX(v0, thetaRad, dCenter);
    }

    /**
     * Computes an <b>initial guess</b> for the hood angle using a drag-free
     * (vacuum) parabola through three constraint points.  The drag-aware outer
     * binary search in {@link #calculatePhysicsWithDrag} refines this guess.
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
     * Simulates trajectory with drag, with explicit ground-frame initial velocity
     * components.  Allows adding the robot's radial velocity to the ball's
     * horizontal speed for accurate flight-time estimation while moving.
     */
    private static double simulateFlightTimeComponents(double vx0, double vy0, double targetXM) {
        double vx = vx0;
        double vy = vy0;
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
     * Prints the full precomputed shooter table to stdout.
     * Called from the build-time unit test so the table is visible on every
     * {@code ./gradlew build}.  Must be called after {@link #precompute()}.
     */
    public static void printTable() {
        System.out.println();
        System.out.println("┌─────────┬──────────┬──────────┬───────┐");
        System.out.println("│ dist(m) │  RPM     │ hood(°)  │ valid │");
        System.out.println("├─────────┼──────────┼──────────┼───────┤");
        for (double dist = SuperstructureConstants.MIN_SHOOT_RANGE_M;
             dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M + 1e-9;
             dist += PRECOMPUTE_STEP_M) {
            double rpm  = RPM_TABLE.get(dist);
            double hood = ANGLE_TABLE.get(dist);
            boolean ok  = VALID_TABLE.get(dist) >= 0.5;
            System.out.printf("│  %5.2f  │ %8.1f │  %6.2f  │   %s   │%n",
                    dist, rpm, hood, ok ? "✓" : "✗");
        }
        System.out.println("└─────────┴──────────┴──────────┴───────┘");
        System.out.println();
    }

    /**
     * Returns a setpoint by interpolating the empirical lookup tables.
     * The {@link InterpolatingDoubleTreeMap} clamps to the nearest bound when
     * queried outside the populated range.
     *
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
    public static double rpmToLaunchSpeed(double rpm) {
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
    public static double v0ToRPM(double v0) {
        return v0 * 60.0 * Shooter.FLYWHEEL_GEAR_RATIO
               / (2.0 * Math.PI * Shooter.FLYWHEEL_WHEEL_RADIUS_M * Shooter.FLYWHEEL_EFFICIENCY);
    }
}
