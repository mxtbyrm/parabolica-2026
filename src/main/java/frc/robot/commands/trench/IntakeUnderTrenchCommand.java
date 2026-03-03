package frc.robot.commands.trench;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Runs the intake roller while the robot drives under the TRENCH structure.
 *
 * <p>This command drives the {@link IntakeSubsystem} directly — it does not
 * interact with the Superstructure state machine.  The intake deploy arm
 * is deployed on initialize and stowed on end; the roller runs throughout.
 *
 * <p><b>Usage in auto:</b>
 * <pre>
 *   Commands.deadline(new IntakeUnderTrenchCommand(intake), followTrenchPath)
 * </pre>
 */
public class IntakeUnderTrenchCommand extends Command {

    private final IntakeSubsystem m_intake;

    /**
     * Constructs an IntakeUnderTrenchCommand.
     *
     * @param intake The intake subsystem.
     */
    public IntakeUnderTrenchCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.deploy();
    }

    @Override
    public void execute() {
        m_intake.runRoller();
    }

    @Override
    public boolean isFinished() {
        return false; // Ended by button release, deadline, or ball-full detection.
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopRoller();
        m_intake.stow();
    }
}
