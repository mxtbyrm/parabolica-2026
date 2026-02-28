package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurretSubsystem;

/**
 * Drives the turret back to its forward-facing home position (0°).
 *
 * <h2>Homing Strategy</h2>
 * <p>There is no limit switch.  The encoder is zeroed once at boot by the
 * {@link TurretSubsystem} constructor while the turret is physically at its
 * forward-facing position.  This command simply commands {@code setAngle(0°)}
 * and waits for the turret to arrive — it does <em>not</em> re-zero the encoder,
 * which would destroy the reference established at boot.
 *
 * <p>Use this after any state where the turret may have been left off-centre
 * (e.g. end of a match, or test-mode direct drive).  The turret will physically
 * rotate back to forward-facing and hold there until a new command takes over.
 *
 * <p>If the encoder has been corrupted (e.g. motor replaced, brownout during
 * manual movement) physically aim the turret forward, then call
 * {@link TurretSubsystem#zeroPosition()} directly from a dashboard button to
 * re-establish the reference before running this command.
 *
 * <p>Bind to {@code Back + A} on the operator controller.
 */
public class HomeTurretCommand extends Command {

    private final TurretSubsystem m_turret;

    /**
     * Constructs a HomeTurretCommand.
     *
     * @param turret The turret subsystem to home.
     */
    public HomeTurretCommand(TurretSubsystem turret) {
        m_turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_turret.setAngle(0.0);   // drive to encoder 0 = physical forward (set at boot)
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return m_turret.isAligned();  // finishes once the turret has arrived at 0°
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_turret.stop();  // interrupted mid-travel — release motor so default command can take over
        }
        // not interrupted = arrived at home; MotionMagic holds position until default command overrides
    }
}
