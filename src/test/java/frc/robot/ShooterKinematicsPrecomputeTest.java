package frc.robot;

import org.junit.jupiter.api.Test;

import frc.robot.util.ShooterKinematics;

public class ShooterKinematicsPrecomputeTest {

    @Test
    void precomputeTiming() {
        long start  = System.currentTimeMillis();
        ShooterKinematics.precompute();
        long elapsed = System.currentTimeMillis() - start;

        System.out.printf("[Test] precompute() finished in %d ms%n", elapsed);
    }
}
