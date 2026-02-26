package frc.robot.commands.trench;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Attempts to intake FUEL while the robot drives under the TRENCH structure.
 *
 * <p>The TRENCH clears 22.125 in (with a 1-in safety margin →
 * {@link frc.robot.Constants.TrenchConstants#TRENCH_MAX_ROBOT_HEIGHT_M}), so
 * only the deployed <em>roller</em> can collect balls — the full deploy pivot
 * cannot extend.  This command places the Superstructure into a special intake
 * posture where the roller runs at a reduced height.
 *
 * <p><b>Hardware note:</b> Before using this command, verify that the deployed
 * intake roller height does not exceed the TRENCH clearance height.  If the
 * intake cannot safely deploy under the TRENCH, use
 * {@link PassThroughTrenchCommand} instead and intake outside the TRENCH only.
 *
 * <h2>Execution Sequence</h2>
 * <ol>
 *   <li><b>Initialize</b> — Requests {@link RobotState#INTAKING}.  The Superstructure
 *       will deploy the intake and run the spindexer.  If the robot's intake exceeds
 *       the TRENCH height when deployed, reconfigure DEPLOY_DEPLOYED_DEG before use.</li>
 *   <li><b>Execute</b> — Holds INTAKING state.</li>
 *   <li><b>IsFinished</b> — Never finishes on its own; end via button release or deadline.</li>
 *   <li><b>End</b> — Returns to {@link RobotState#STOWED}.</li>
 * </ol>
 *
 * <p><b>Usage in auto:</b>
 * <pre>
 *   Commands.deadline(new IntakeUnderTrenchCommand(superstructure), followTrenchPath)
 * </pre>
 */
public class IntakeUnderTrenchCommand extends Command {

    private final Superstructure m_superstructure;

    /**
     * Constructs an IntakeUnderTrenchCommand.
     *
     * @param superstructure The central superstructure state machine.
     */
    public IntakeUnderTrenchCommand(Superstructure superstructure) {
        m_superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        m_superstructure.requestState(RobotState.INTAKING);
    }

    @Override
    public void execute() {
        if (m_superstructure.getState() != RobotState.INTAKING) {
            m_superstructure.requestState(RobotState.INTAKING);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Ended by button release, deadline, or ball-full detection.
    }

    @Override
    public void end(boolean interrupted) {
        m_superstructure.requestState(RobotState.STOWED);
    }
}
