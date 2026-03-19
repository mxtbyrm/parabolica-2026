package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Holds the intake rack at the intermediate agitate position while the button
 * is held, running the roller at a reduced duty cycle to gently jostle balls.
 *
 * <p>While held:
 * <ul>
 *   <li>Rack moves to and stays at {@link frc.robot.Constants.Intake#DEPLOY_AGITATE_ROT}.</li>
 *   <li>Roller runs at 20% duty cycle.</li>
 * </ul>
 *
 * <p>On release the rack returns to fully deployed and the roller stops,
 * leaving the intake ready for normal operation.
 */
public class IntakeAgitateCommand extends Command {

    private static final double AGITATE_ROLLER_PERCENT = 0.20;

    private final IntakeSubsystem m_intake;

    /**
     * @param intake The intake subsystem.
     */
    public IntakeAgitateCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.agitate();
        m_intake.runRollerAt(AGITATE_ROLLER_PERCENT);
    }

    @Override
    public void execute() {
        // Hold position — agitate() and runRollerAt() are idempotent motor commands;
        // re-issuing each loop keeps the control request alive under scheduler jitter.
        m_intake.agitate();
        m_intake.runRollerAt(AGITATE_ROLLER_PERCENT);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until the operator releases the button.
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopRoller();
        m_intake.deploy(); // Return to fully deployed so the intake is ready.
    }
}
