package frc.robot;

import org.junit.jupiter.api.Test;

import frc.robot.util.ShooterKinematics;

public class ShooterKinematicsPrecomputeTest {

    @Test
    void precomputeAndPrintTable() {
        ShooterKinematics.precompute();
        ShooterKinematics.printTable();
    }
}
