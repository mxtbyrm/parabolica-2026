package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurretSubsystem;

/**
 * Sets the turret encoder to zero, declaring the current position as home.
 *
 * <h2>Homing Strategy</h2>
 * <p>There is no limit switch and no hard-stop stall routine.  The robot is
 * always powered on with the turret physically at its forward-facing home
 * position, so {@link TurretSubsystem#zeroPosition()} is the only operation
 * required.  The command finishes in the same scheduler tick it starts.
 *
 * <p>Run once per power-on if the encoder needs to be explicitly confirmed
 * (e.g. after manually rotating the turret while disabled).  Under normal
 * operation the {@link TurretSubsystem} constructor already calls
 * {@code zeroPosition()} at boot, so this command is a manual reset only.
 *
 * <p>Bind to {@code Back + A} on the operator controller.
 */
public class HomeTurretCommand extends Command {

    private final TurretSubsystem m_turret;
    private boolean m_done = false;

    /**
     * Constructs a HomeTurretCommand.
     *
     * @param turret The turret subsystem whose encoder will be zeroed.
     */
    public HomeTurretCommand(TurretSubsystem turret) {
        m_turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_done = false;
        m_turret.zeroPosition(); // current position is home
        m_turret.setAngle(0.0);  // hold forward-facing via closed loop
        m_done = true;
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return m_done;
    }

    @Override
    public void end(boolean interrupted) {}
}
