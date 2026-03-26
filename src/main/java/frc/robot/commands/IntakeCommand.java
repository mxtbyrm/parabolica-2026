package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Deploys the intake arm and runs the roller to collect FUEL from the floor.
 *
 * <p>This command drives the {@link IntakeSubsystem} directly — it does not
 * interact with the Superstructure state machine.  Intake deploy/stow and
 * roller are fully independent of the scoring pipeline.
 *
 * <p>Intended for autonomous PathPlanner named-command use.  In teleop the
 * operator controls deploy and roller independently via separate bindings.
 */
public class IntakeCommand extends Command {

    private final IntakeSubsystem m_intake;

    /**
     * Constructs an IntakeCommand.
     *
     * @param intake The intake subsystem.
     */
    public IntakeCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.runRoller();
        }

    @Override
    public void execute() {
        m_intake.runRoller();
    }

    /** This command runs until interrupted (button release or deadline). */
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopRoller();
    }
}
