package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Cycles the intake arm between agitate and deployed positions while the button
 * is held, running the roller at a reduced duty cycle during the agitate phase
 * to gently jostle balls.
 *
 * <p>Cycle (repeats until button released):
 * <ol>
 *   <li>Arm moves to agitate position, roller at 20% — for {@code AGITATE_DURATION_S}.</li>
 *   <li>Arm returns to fully deployed, roller stops — for {@code AGITATE_INTERVAL_S}.</li>
 * </ol>
 *
 * <p>On release the rack returns to fully deployed and the roller stops.
 */
public class IntakeAgitateCommand extends Command {

    private static final double AGITATE_ROLLER_PERCENT = 0.20;
    private static final double AGITATE_DURATION_S     = 0.35;
    private static final double AGITATE_INTERVAL_S     = 1.75;

    private final IntakeSubsystem m_intake;
    private final Timer           m_timer    = new Timer();
    private boolean               m_agitating = false;

    public IntakeAgitateCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Start immediately in agitate phase.
        m_agitating = true;
        m_intake.agitate();
        m_intake.runRollerAt(AGITATE_ROLLER_PERCENT);
        m_timer.restart();
    }

    @Override
    public void execute() {
        if (m_agitating) {
            m_intake.agitate();
            m_intake.runRollerAt(AGITATE_ROLLER_PERCENT);
            if (m_timer.hasElapsed(AGITATE_DURATION_S)) {
                m_agitating = false;
                m_intake.deploy();
                m_intake.stopRoller();
                m_timer.restart();
            }
        } else {
            m_intake.deploy();
            if (m_timer.hasElapsed(AGITATE_INTERVAL_S)) {
                m_agitating = true;
                m_intake.agitate();
                m_intake.runRollerAt(AGITATE_ROLLER_PERCENT);
                m_timer.restart();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopRoller();
        m_intake.deploy();
    }
}
