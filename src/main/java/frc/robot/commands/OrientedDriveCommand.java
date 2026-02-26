package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Robot-centric drive command at reduced speed for navigating tight areas.
 *
 * <p>When active, this command overrides the default field-centric drive with a
 * <em>robot-centric</em> drive at {@link #SPEED_SCALE} (30%) of maximum speed.
 * Robot-centric control is more predictable in confined spaces such as the TRENCH
 * approach corridor, where field-orientation may be disorienting.
 *
 * <h2>Control Scheme</h2>
 * <ul>
 *   <li>Left stick Y → forward / backward (robot-relative)</li>
 *   <li>Left stick X → strafe left / right (robot-relative)</li>
 *   <li>Right stick X → rotation</li>
 * </ul>
 *
 * <h2>Speed Scaling</h2>
 * <p>All velocity and angular rate inputs are scaled by {@link #SPEED_SCALE} (0.30)
 * before being sent to the drivetrain.  This gives fine-grained control at low speed.
 *
 * <p>Bind to a controller button via {@code whileTrue()} so the mode is only active
 * while the button is held:
 * <pre>
 *   joystick.leftStick().whileTrue(new OrientedDriveCommand(drivetrain, ...));
 * </pre>
 */
public class OrientedDriveCommand extends Command {

    /** Fraction of maximum speed applied in oriented drive mode. */
    public static final double SPEED_SCALE = 0.30;

    // =========================================================================
    // Dependencies
    // =========================================================================

    private final CommandSwerveDrivetrain m_drivetrain;
    private final DoubleSupplier          m_leftY;
    private final DoubleSupplier          m_leftX;
    private final DoubleSupplier          m_rightX;
    private final double                  m_maxSpeed;
    private final double                  m_maxAngularRate;

    // Reused swerve request — avoids heap allocation each loop.
    private final SwerveRequest.RobotCentric m_robotCentricReq =
            new SwerveRequest.RobotCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Constructs an OrientedDriveCommand.
     *
     * @param drivetrain     The swerve drivetrain subsystem.
     * @param leftY          Supplier for the left stick Y axis (forward/back input, [-1, 1]).
     * @param leftX          Supplier for the left stick X axis (strafe input, [-1, 1]).
     * @param rightX         Supplier for the right stick X axis (rotation input, [-1, 1]).
     * @param maxSpeed       Maximum robot translation speed in m/s (from TunerConstants).
     * @param maxAngularRate Maximum angular rate in rad/s (used in default drive command).
     */
    public OrientedDriveCommand(CommandSwerveDrivetrain drivetrain,
                                 DoubleSupplier leftY,
                                 DoubleSupplier leftX,
                                 DoubleSupplier rightX,
                                 double maxSpeed,
                                 double maxAngularRate) {
        m_drivetrain     = drivetrain;
        m_leftY          = leftY;
        m_leftX          = leftX;
        m_rightX         = rightX;
        m_maxSpeed       = maxSpeed;
        m_maxAngularRate = maxAngularRate;
        addRequirements(drivetrain);
    }

    // =========================================================================
    // Command Lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        // Nothing special needed on entry.
    }

    @Override
    public void execute() {
        double scaledMax  = m_maxSpeed       * SPEED_SCALE;
        double scaledOmeg = m_maxAngularRate * SPEED_SCALE;

        m_drivetrain.setControl(
            m_robotCentricReq
                .withVelocityX(-m_leftY.getAsDouble()  * scaledMax)
                .withVelocityY(-m_leftX.getAsDouble()  * scaledMax)
                .withRotationalRate(-m_rightX.getAsDouble() * scaledOmeg)
        );
    }

    @Override
    public boolean isFinished() {
        return false; // Held continuously while button is pressed.
    }

    @Override
    public void end(boolean interrupted) {
        // The default drive command will take over on the next scheduler cycle.
    }
}
