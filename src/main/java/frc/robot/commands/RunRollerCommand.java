package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Runs the intake roller while the operator holds the bound button.
 *
 * <p>The roller will only spin if the intake arm is already in the deployed
 * position ({@link IntakeSubsystem#isDeployed()} returns {@code true}).
 * If the arm is stowed or still traveling, the roller stays stopped — the guard
 * lives inside {@link IntakeSubsystem#runRoller()} and is re-evaluated every loop,
 * so no explicit sequencing is needed.
 *
 * <p>This command does <em>not</em> move the intake arm.  To deploy the arm,
 * use {@link IntakeCommand} (which requests {@code RobotState.INTAKING}) or
 * bind a separate deploy command.
 *
 * <p>Bind to a controller button as {@code whileTrue}.  The roller stops
 * automatically when the button is released or the command is interrupted.
 */
public class RunRollerCommand extends Command {

    private final IntakeSubsystem m_intake;

    /**
     * Constructs a RunRollerCommand.
     *
     * @param intake The intake subsystem whose roller will be driven.
     */
    public RunRollerCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_intake.runRoller();   // no-op if arm is not yet deployed
    }

    @Override
    public boolean isFinished() {
        return false;           // runs until the operator releases the button
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopRoller();
    }
}
