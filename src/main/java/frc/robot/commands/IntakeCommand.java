package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Deploys the intake and runs the spindexer to collect FUEL from the floor.
 *
 * <p>This command delegates all mechanism coordination to the
 * {@link Superstructure} state machine by requesting {@link RobotState#INTAKING}.
 * When the command ends (operator releases trigger or it is interrupted), the
 * Superstructure transitions back to {@link RobotState#STOWED}.
 *
 * <p>Intended for use as a {@code whileTrue} binding on a controller trigger.
 */
public class IntakeCommand extends Command {

    private final Superstructure m_superstructure;

    /**
     * Constructs an IntakeCommand.
     *
     * @param superstructure The superstructure state machine.  Required.
     */
    public IntakeCommand(Superstructure superstructure) {
        m_superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        m_superstructure.requestState(RobotState.INTAKING);
    }

    /** This command runs until the operator releases the trigger. */
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_superstructure.requestState(RobotState.STOWED);
    }
}
