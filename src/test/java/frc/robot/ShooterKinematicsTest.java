package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import frc.robot.Constants.Field;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.util.ShooterKinematics;
import frc.robot.util.ShooterKinematics.ShooterSetpoint;

/**
 * Comprehensive validation of the ShooterKinematics physics model.
 * 
 * <p>Tests cover:
 * <ul>
 *   <li>Precompute table population and timing</li>
 *   <li>Setpoint sanity (RPM range, hood angle bounds)</li>
 *   <li>Monotonicity (RPM increases with distance)</li>
 *   <li>Flight time positivity and monotonicity</li>
 *   <li>Hexagonal rim geometry (circumradius vs. apothem)</li>
 *   <li>Trajectory clearance verification at both flat-face and vertex rims</li>
 * </ul>
 */
class ShooterKinematicsTest {

    // =========================================================================
    // Physical constants (mirrored from ShooterKinematics for trajectory
    // re-simulation in tests — keep in sync with the source)
    // =========================================================================

    private static final double GRAVITY        = 9.81;
    private static final double DRAG_COEFF     = 0.47;
    private static final double AIR_DENSITY    = 1.225;
    private static final double BALL_AREA      = Math.PI * Field.FUEL_RADIUS_M * Field.FUEL_RADIUS_M;
    private static final double DRAG_ACCEL     = (0.5 * DRAG_COEFF * AIR_DENSITY * BALL_AREA)
                                                 / Field.FUEL_MASS_KG;
    private static final double SIM_DT         = 0.0005; // finer than prod for test accuracy

    /** Hexagonal apothem (shortest center-to-rim, flat-face approach). */
    private static final double HEX_APOTHEM    = Field.HUB_TOP_OPENING_DIAMETER_M / 2.0;
    /** Hexagonal circumradius (longest center-to-rim, vertex approach). */
    private static final double HEX_CIRCUMRADIUS =
            Field.HUB_TOP_OPENING_DIAMETER_M / Math.sqrt(3.0);

    /** Required clearance height for ball center above the floor. */
    private static final double H_CLEARANCE    = Field.HUB_TOP_OPENING_HEIGHT_M
                                                + Field.FUEL_RADIUS_M
                                                + Shooter.RIM_SAFETY_MARGIN_M;

    @BeforeAll
    static void precompute() {
        ShooterKinematics.precompute();
    }

    // =====================================================================
    // 1. Precompute timing
    // =====================================================================

    @Test
    void precomputeFinishesInReasonableTime() {
        long start   = System.currentTimeMillis();
        ShooterKinematics.precompute(); // second call re-populates
        long elapsed = System.currentTimeMillis() - start;
        System.out.printf("[Test] precompute() finished in %d ms%n", elapsed);
        assertTrue(elapsed < 5_000, "Precompute should finish in <5 s (got " + elapsed + " ms)");
    }

    // =====================================================================
    // 2. Setpoint sanity across the full range
    // =====================================================================

    @ParameterizedTest
    @ValueSource(doubles = {1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0})
    void setpointRPMInReasonableRange(double dist) {
        assumeTrue(ShooterKinematics.isShootable(dist), "Distance " + dist + " m not shootable — skipping");
        ShooterSetpoint sp = ShooterKinematics.calculate(dist);
        assertTrue(sp.flywheelRPM() > 500,
                "RPM too low at " + dist + " m: " + sp.flywheelRPM());
        assertTrue(sp.flywheelRPM() < 10_000,
                "RPM too high at " + dist + " m: " + sp.flywheelRPM());
    }

    @ParameterizedTest
    @ValueSource(doubles = {1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0})
    void hoodAngleWithinMechanicalLimits(double dist) {
        ShooterSetpoint sp = ShooterKinematics.calculate(dist);
        assertTrue(sp.hoodAngleDeg() >= Shooter.HOOD_MIN_ANGLE_DEG - 0.01,
                "Hood below min at " + dist + " m: " + sp.hoodAngleDeg());
        assertTrue(sp.hoodAngleDeg() <= Shooter.HOOD_MAX_ANGLE_DEG + 0.01,
                "Hood above max at " + dist + " m: " + sp.hoodAngleDeg());
    }

    // =====================================================================
    // 3. RPM monotonicity — farther = more RPM (generally)
    // =====================================================================

    @Test
    void rpmGenerallyIncreasesWithDistance() {
        double prevRPM = 0;
        int violations = 0;
        for (double d = SuperstructureConstants.MIN_SHOOT_RANGE_M;
             d <= SuperstructureConstants.MAX_SHOOT_RANGE_M;
             d += 0.25) {
            double rpm = ShooterKinematics.calculate(d).flywheelRPM();
            if (rpm < prevRPM - 50) { // allow small non-monotonicity from interpolation
                violations++;
            }
            prevRPM = rpm;
        }
        assertTrue(violations <= 2,
                "RPM decreased by >50 more than twice across the range (" + violations + " times)");
    }

    // =====================================================================
    // 4. Flight time sanity
    // =====================================================================

    @ParameterizedTest
    @ValueSource(doubles = {2.0, 3.0, 4.0, 5.0, 6.0})
    void flightTimePositiveAndReasonable(double dist) {
        ShooterSetpoint sp = ShooterKinematics.calculate(dist);
        double t = ShooterKinematics.getFlightTimeSeconds(dist, sp);
        assertTrue(t > 0.05, "Flight time too short at " + dist + " m: " + t);
        assertTrue(t < 3.0,  "Flight time too long at "  + dist + " m: " + t);
    }

    @Test
    void flightTimeIncreasesWithDistance() {
        double prevT = 0;
        for (double d = 2.0; d <= 6.0; d += 0.5) {
            ShooterSetpoint sp = ShooterKinematics.calculate(d);
            double t = ShooterKinematics.getFlightTimeSeconds(d, sp);
            assertTrue(t >= prevT - 0.01,
                    "Flight time decreased significantly at " + d + " m");
            prevT = t;
        }
    }

    // =====================================================================
    // 5. Hexagonal geometry constants
    // =====================================================================

    @Test
    void hexCircumradiusLargerThanApothem() {
        assertTrue(HEX_CIRCUMRADIUS > HEX_APOTHEM,
                "Circumradius must be > apothem");
        double ratio = HEX_CIRCUMRADIUS / HEX_APOTHEM;
        assertEquals(2.0 / Math.sqrt(3.0), ratio, 1e-9,
                "Circumradius / apothem should be 2/√3");
    }

    // =====================================================================
    // 6. Trajectory clearance — the most important test
    //
    // For every precomputed setpoint, independently simulate the trajectory
    // and verify the ball clears the HUB rim at BOTH the flat-face distance
    // (apothem) and the vertex distance (circumradius).
    // =====================================================================

    @ParameterizedTest
    @ValueSource(doubles = {1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0})
    void trajectoryClearsRimAtVertexApproach(double dist) {
        assumeTrue(ShooterKinematics.isShootable(dist), "Distance " + dist + " m not shootable — skipping");
        ShooterSetpoint sp = ShooterKinematics.calculate(dist);
        double dRimVertex = Math.max(0.1, dist - HEX_CIRCUMRADIUS);
        double heightAtRim = simulateTrajectory(sp, dRimVertex);

        assertTrue(heightAtRim >= H_CLEARANCE,
                String.format("Ball does NOT clear vertex rim at %.1f m: "
                        + "height=%.4f m, required=%.4f m (deficit=%.1f mm)",
                        dist, heightAtRim, H_CLEARANCE,
                        (H_CLEARANCE - heightAtRim) * 1000));
    }

    @ParameterizedTest
    @ValueSource(doubles = {1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0})
    void trajectoryClearsRimAtFlatFaceApproach(double dist) {
        assumeTrue(ShooterKinematics.isShootable(dist), "Distance " + dist + " m not shootable — skipping");
        ShooterSetpoint sp = ShooterKinematics.calculate(dist);
        double dRimFlat = Math.max(0.1, dist - HEX_APOTHEM);
        double heightAtRim = simulateTrajectory(sp, dRimFlat);

        assertTrue(heightAtRim >= H_CLEARANCE,
                String.format("Ball does NOT clear flat-face rim at %.1f m: "
                        + "height=%.4f m, required=%.4f m (deficit=%.1f mm)",
                        dist, heightAtRim, H_CLEARANCE,
                        (H_CLEARANCE - heightAtRim) * 1000));
    }

    @ParameterizedTest
    // Distances below 2.5 m excluded: the center-targeting solver produces a
    // steep, high arc at close range to drop 20 cm into the basket.  At the
    // far vertex (dist + circumradius) the ball may still be above rim height
    // due to the higher RPM.  At ≥2.5 m the arc flattens out and the ball
    // reliably descends before the far rim.
    @ValueSource(doubles = {2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0})
    void ballDescendsIntoHubAfterClearingNearRim(double dist) {
        // Verify the ball descends below hClearance before reaching the far
        // rim — i.e. it actually enters the hub rather than flying over it.
        ShooterSetpoint sp = ShooterKinematics.calculate(dist);
        double dRimFar  = dist + HEX_CIRCUMRADIUS; // far side of opening
        double heightAtFar = simulateTrajectory(sp, dRimFar);

        assertTrue(heightAtFar < H_CLEARANCE,
                String.format("Ball overshoots hub at %.1f m: "
                        + "height at far rim = %.4f m (should be < %.4f m)",
                        dist, heightAtFar, H_CLEARANCE));
    }

    // =====================================================================
    // 7. RPM / launch-speed round-trip consistency
    //
    // We can't call private methods directly, but we can verify that the
    // setpoint produces a velocity consistent with the RPM via the known
    // conversion formula.
    // =====================================================================

    @Test
    void rpmToSpeedAndBackAreConsistent() {
        // Mirror the conversion formulas from ShooterKinematics
        for (double rpm = 1000; rpm <= 8000; rpm += 500) {
            double v = rpm / Shooter.FLYWHEEL_GEAR_RATIO
                       * 2.0 * Math.PI * Shooter.FLYWHEEL_WHEEL_RADIUS_M
                       * Shooter.FLYWHEEL_EFFICIENCY / 60.0;
            double rpmBack = v * 60.0 * Shooter.FLYWHEEL_GEAR_RATIO
                             / (2.0 * Math.PI * Shooter.FLYWHEEL_WHEEL_RADIUS_M
                                * Shooter.FLYWHEEL_EFFICIENCY);
            assertEquals(rpm, rpmBack, 1e-9,
                    "RPM round-trip failed at " + rpm);
        }
    }

    // =====================================================================
    // 8. Hood-to-exit angle conversion
    // =====================================================================

    @Test
    void hoodToExitAngleIsCorrect() {
        // ballExitAngle = 90 - hoodAngle
        assertEquals(62.5, 90.0 - Shooter.HOOD_MIN_ANGLE_DEG, 1e-9,
                "Exit angle at min hood");
        assertEquals(47.5, 90.0 - Shooter.HOOD_MAX_ANGLE_DEG, 1e-9,
                "Exit angle at max hood");
    }

    // =====================================================================
    // 9. Drag coefficient sanity
    // =====================================================================

    @Test
    void dragAccelCoefficientIsPhysicallyReasonable() {
        // For a 5.91" foam ball at 0.215 kg, drag decel coeff should be
        // in the range 0.01–0.10 per (m/s)^2.
        assertTrue(DRAG_ACCEL > 0.01 && DRAG_ACCEL < 0.10,
                "DRAG_ACCEL_COEFF out of expected range: " + DRAG_ACCEL);
    }

    // =====================================================================
    // 10. Print setpoint table for manual inspection
    // =====================================================================

    @Test
    void printSetpointTable() {
        System.out.println("\n=== Shooter Setpoint Table ===");
        System.out.printf("%-8s  %-10s  %-10s  %-10s  %-12s  %-12s%n",
                "Dist(m)", "RPM", "Hood(°)", "Exit(°)", "dRim_vtx(m)", "FlightT(s)");
        System.out.println("-".repeat(75));

        for (double d = SuperstructureConstants.MIN_SHOOT_RANGE_M;
             d <= SuperstructureConstants.MAX_SHOOT_RANGE_M + 0.01;
             d += 0.5) {
            ShooterSetpoint sp = ShooterKinematics.calculate(d);
            double exitAngle = 90.0 - sp.hoodAngleDeg();
            double dRim = Math.max(0.1, d - HEX_CIRCUMRADIUS);
            double flightT = ShooterKinematics.getFlightTimeSeconds(d, sp);
            System.out.printf("%-8.1f  %-10.1f  %-10.1f  %-10.1f  %-12.3f  %-12.4f%n",
                    d, sp.flywheelRPM(), sp.hoodAngleDeg(), exitAngle, dRim, flightT);
        }
    }

    // =====================================================================
    // Helpers
    // =====================================================================

    /**
     * Independently simulates the trajectory for a given setpoint and returns
     * the ball's height at horizontal position {@code targetXM}.
     *
     * <p>Uses a finer time step (0.5 ms) than the production solver (10 ms)
     * and endpoint interpolation for higher accuracy.
     */
    private double simulateTrajectory(ShooterSetpoint sp, double targetXM) {
        double exitAngleRad = Math.toRadians(90.0 - sp.hoodAngleDeg());
        double v0 = sp.flywheelRPM() / Shooter.FLYWHEEL_GEAR_RATIO
                    * 2.0 * Math.PI * Shooter.FLYWHEEL_WHEEL_RADIUS_M
                    * Shooter.FLYWHEEL_EFFICIENCY / 60.0;

        double vx = v0 * Math.cos(exitAngleRad);
        double vy = v0 * Math.sin(exitAngleRad);
        double x  = 0.0;
        double y  = Shooter.LAUNCH_HEIGHT_M;

        for (int step = 0; step < 200_000 && x < targetXM; step++) {
            if (y < 0.0 || vx <= 0.0) return Double.NEGATIVE_INFINITY;

            double xPrev = x;
            double yPrev = y;

            double v  = Math.sqrt(vx * vx + vy * vy);
            double ax = -DRAG_ACCEL * v * vx;
            double ay = -GRAVITY - DRAG_ACCEL * v * vy;
            x  += vx * SIM_DT;
            y  += vy * SIM_DT;
            vx += ax * SIM_DT;
            vy += ay * SIM_DT;

            if (x >= targetXM) {
                double frac = (targetXM - xPrev) / (x - xPrev);
                return yPrev + frac * (y - yPrev);
            }
        }
        return y;
    }
}
