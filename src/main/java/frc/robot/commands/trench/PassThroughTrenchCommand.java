package frc.robot.commands.trench;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TrenchTraversalManager;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Drives the robot through the TRENCH while ensuring all overhead mechanisms are stowed.
 *
 * <h2>Execution Sequence</h2>
 * <ol>
 *   <li><b>Initialize</b> — Immediately requests {@link RobotState#TRAVERSING_TRENCH}
 *       so all mechanisms stow before the robot enters the structure.</li>
 *   <li><b>Execute</b> — Waits while the robot remains inside the TRENCH zone
 *       (or the approach margin).  The drivetrain is driven by the default drive
 *       command or a PathPlanner path in parallel — this command only controls the
 *       Superstructure.</li>
 *   <li><b>IsFinished</b> — Returns {@code true} once the robot has fully exited the
 *       TRENCH zone according to {@link TrenchTraversalManager#isNearOrInsideTrench}.</li>
 *   <li><b>End (natural)</b> — Returns the Superstructure to {@link RobotState#STOWED}.</li>
 *   <li><b>End (interrupted)</b> — Also returns to {@link RobotState#STOWED}.</li>
 * </ol>
 *
 * <p><b>Usage in auto:</b> compose with a PathPlanner path that drives the robot through
 * the TRENCH as a {@code deadline} group so the path ends when the path completes:
 * <pre>
 *   Commands.deadline(new PassThroughTrenchCommand(superstructure), followTrenchPath)
 * </pre>
 *
 * <p><b>Usage in teleop:</b> bind to a button via {@code whileTrue()} so the driver can
 * manually trigger TRENCH mode.  {@link TrenchTraversalManager} will also trigger this
 * state automatically when the robot approaches a TRENCH.
 */
public class PassThroughTrenchCommand extends Command {

    private final Superstructure m_superstructure;

    /**
     * Constructs a PassThroughTrenchCommand.
     *
     * @param superstructure The central superstructure state machine.
     */
    public PassThroughTrenchCommand(Superstructure superstructure) {
        m_superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        m_superstructure.requestState(RobotState.TRAVERSING_TRENCH);
    }

    @Override
    public void execute() {
        // Keep requesting TRAVERSING_TRENCH in case another command tried to change state.
        if (m_superstructure.getState() != RobotState.TRAVERSING_TRENCH) {
            m_superstructure.requestState(RobotState.TRAVERSING_TRENCH);
        }
    }

    @Override
    public boolean isFinished() {
        // This command relies on the caller (deadline group or button) to end it.
        // It does not self-terminate — the driver or path planner decides when done.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_superstructure.requestState(RobotState.STOWED);
    }
}
