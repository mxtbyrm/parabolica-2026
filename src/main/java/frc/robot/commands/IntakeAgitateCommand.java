package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Oscillates the intake rack between the fully deployed position and the
 * intermediate agitate position to jostle balls through the intake faster.
 *
 * <p>While this command runs the rack cycles:
 * <pre>
 *   DEPLOYED → AGITATE → DEPLOYED → AGITATE → ...
 * </pre>
 * The roller is not touched — run it separately if intake is active.
 *
 * <p>On end (button release) the rack returns to the fully deployed position
 * so the intake remains ready for the next ball.
 *
 * <p>The agitate position ({@link frc.robot.Constants.Intake#DEPLOY_AGITATE_ROT})
 * is NOT fully stowed — it is an intermediate point that partially retracts the
 * rack to create a jostling motion without losing ground contact.
 */
public class IntakeAgitateCommand extends Command {

    private final IntakeSubsystem m_intake;
    private boolean m_headingToAgitate;

    /**
     * @param intake The intake subsystem.
     */
    public IntakeAgitateCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Start by moving toward the agitate (partial retract) position.
        m_headingToAgitate = true;
        m_intake.agitate();
    }

    @Override
    public void execute() {
        if (m_headingToAgitate) {
            if (m_intake.isAtAgitatePosition()) {
                m_headingToAgitate = false;
                m_intake.deploy();
            }
        } else {
            if (m_intake.isDeployed()) {
                m_headingToAgitate = true;
                m_intake.agitate();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until the operator releases the button.
    }

    @Override
    public void end(boolean interrupted) {
        // Return to fully deployed so the intake is ready for the next ball.
        m_intake.deploy();
    }
}
