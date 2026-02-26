package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;

/**
 * Autonomous shoot command that wraps the full shoot sequence with a timeout.
 *
 * <p>This command is intended for use in PathPlanner auto routines via
 * {@code NamedCommands.registerCommand("Shoot", new AutoShootCommand(...)}.
 * It behaves identically to {@link ShootCommand} but adds a configurable timeout
 * so that an auto path does not stall indefinitely if the robot cannot achieve
 * shot readiness (e.g. vision target lost, hub inactive).
 *
 * <h2>Execution Sequence</h2>
 * <ol>
 *   <li><b>Initialize</b> — Starts the timeout timer, then delegates to
 *       {@link ShootCommand#initialize()}.</li>
 *   <li><b>Execute</b> — Delegates each loop to {@link ShootCommand#execute()}.
 *       The command also ends if {@link #m_timeout} seconds have elapsed, regardless
 *       of shot completion.</li>
 *   <li><b>IsFinished</b> — {@code true} when the inner command finishes OR the
 *       timeout expires.</li>
 *   <li><b>End</b> — Delegates to {@link ShootCommand#end(boolean)} and returns the
 *       Superstructure to {@link RobotState#STOWED}.</li>
 * </ol>
 *
 * <p>Default timeout is {@link #DEFAULT_TIMEOUT_S} (3.0 seconds).
 */
public class AutoShootCommand extends Command {

    /** Default auto-shoot timeout in seconds. */
    public static final double DEFAULT_TIMEOUT_S = 3.0;

    // =========================================================================
    // Fields
    // =========================================================================

    private final ShootCommand m_innerShootCommand;
    private final double       m_timeout;
    private final Timer        m_timer = new Timer();

    // =========================================================================
    // Constructors
    // =========================================================================

    /**
     * Constructs an AutoShootCommand with the default timeout.
     *
     * @param superstructure The superstructure state machine.
     * @param vision         The vision subsystem.
     * @param drivetrain     The swerve drivetrain.
     */
    public AutoShootCommand(Superstructure superstructure,
                             VisionSubsystem vision,
                             CommandSwerveDrivetrain drivetrain) {
        this(superstructure, vision, drivetrain, DEFAULT_TIMEOUT_S);
    }

    /**
     * Constructs an AutoShootCommand with a custom timeout.
     *
     * @param superstructure The superstructure state machine.
     * @param vision         The vision subsystem.
     * @param drivetrain     The swerve drivetrain.
     * @param timeoutSeconds Maximum time in seconds before the command ends.
     */
    public AutoShootCommand(Superstructure superstructure,
                             VisionSubsystem vision,
                             CommandSwerveDrivetrain drivetrain,
                             double timeoutSeconds) {
        m_innerShootCommand = new ShootCommand(superstructure, vision, drivetrain);
        m_timeout           = timeoutSeconds;
        // Inherit requirements from the inner command.
        addRequirements(m_innerShootCommand.getRequirements().toArray(new edu.wpi.first.wpilibj2.command.Subsystem[0]));
    }

    // =========================================================================
    // Command Lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        m_timer.restart();
        m_innerShootCommand.initialize();
    }

    @Override
    public void execute() {
        m_innerShootCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return m_innerShootCommand.isFinished() || m_timer.hasElapsed(m_timeout);
    }

    @Override
    public void end(boolean interrupted) {
        boolean timedOut = m_timer.hasElapsed(m_timeout);
        // Treat timeout as an interruption so the superstructure returns to STOWED.
        m_innerShootCommand.end(interrupted || timedOut);
        m_timer.stop();
    }
}
