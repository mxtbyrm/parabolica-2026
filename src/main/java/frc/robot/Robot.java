package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.util.ShooterKinematics;

/**
 * Top-level robot class.  Bootstraps all subsystems via {@link RobotContainer}
 * and drives the {@link CommandScheduler} each loop.
 *
 * <h2>Vision Architecture</h2>
 * <p>All Limelight / MegaTag2 pose-estimation logic lives in
 * {@link frc.robot.subsystems.VisionSubsystem}, which is constructed inside
 * {@link RobotContainer} and registered as a WPILib subsystem.  The
 * CommandScheduler calls {@code VisionSubsystem.periodic()} every 20 ms, which:
 * <ol>
 *   <li>Provides the robot's current heading to MegaTag2 (gyro-locked yaw).</li>
 *   <li>Calls {@code getBotPoseEstimate_wpiBlue/Red_MegaTag2()} for the active
 *       alliance and injects the result into the swerve pose estimator with
 *       dynamically-scaled standard deviations:
 *       {@code σ_xy = 0.4 + 0.3 × avgTagDist² / tagCount}.</li>
 *   <li>Rejects estimates when rotational speed exceeds 2 rot/s (gyro coupling
 *       artifact at high omega) or fewer than 1 tag is visible.</li>
 *   <li>Selects the best HUB or TRENCH AprilTag, rejects high-ambiguity
 *       detections (ambiguity &gt; 0.15), and exposes distance + tx for the
 *       turret and flywheel.</li>
 * </ol>
 * <p>No vision code belongs in this file.
 *
 * <h2>Logging</h2>
 * <p>{@link SignalLogger} writes all CTRE Phoenix 6 signals to {@code .hoot}
 * files on the roboRIO USB drive.  {@link DataLogManager} additionally captures
 * WPILib DataLog entries (NetworkTables, DS status) to the same media.
 * {@link HootAutoReplay} can replay timestamps and joystick data from a
 * recorded session for deterministic log analysis.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /** Replays timestamps and joystick data from .hoot log files for analysis. */
    private final HootAutoReplay m_hootReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        // Precompute shooter physics table before anything else.
        // Runs the Euler solver across the full range once while disabled;
        // all match-time calls to ShooterKinematics.calculate() then just interpolate.
        ShooterKinematics.precompute();

        // Start CTRE SignalLogger (writes .hoot files to USB / internal flash).
        SignalLogger.start();

        // Mirror WPILib DataLog entries to the same log directory so both
        // Phoenix signals and NT/DS data end up in one place.
        DataLogManager.start();

        // Automatically capture DriverStation data (match info, enabled state, etc.).
        DriverStation.startDataLog(DataLogManager.getLog());

        m_robotContainer = new RobotContainer();
    }

    // =========================================================================
    // Robot-wide periodic  (runs every 20 ms, all modes)
    // =========================================================================

    @Override
    public void robotPeriodic() {
        // Replay log timestamps / joystick data when analysing .hoot recordings.
        m_hootReplay.update();

        // Drive the WPILib command scheduler — runs all subsystem periodic()
        // methods (including VisionSubsystem) and executes scheduled commands.
        CommandScheduler.getInstance().run();
    }

    // =========================================================================
    // Disabled
    // =========================================================================

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    // =========================================================================
    // Autonomous
    // =========================================================================

    @Override
    public void autonomousInit() {
        m_robotContainer.prepareForMatch(); // preset ball count from SmartDashboard
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    // =========================================================================
    // Teleop
    // =========================================================================

    @Override
    public void teleopInit() {
        // Cancel auto path if the driver takes over before it finishes.
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        m_robotContainer.prepareForMatch(); // re-sync ball count at teleop start
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    // =========================================================================
    // Test
    // =========================================================================

    @Override
    public void testInit() {
        // Clear all running commands so SysId routines start from a clean state.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    // =========================================================================
    // Simulation
    // =========================================================================

    @Override
    public void simulationPeriodic() {}
}
