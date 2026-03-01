package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

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
 * <p>Requires both {@link TurretSubsystem} and {@link Superstructure} so that
 * any running {@link ShootCommand} (which also requires Superstructure) is
 * interrupted, preventing it from fighting this command over turret angle.
 *
 * <p>Bind to {@code Back + A} on the operator controller.
 */
public class HomeTurretCommand extends Command {

    private final TurretSubsystem m_turret;
    private final Superstructure  m_superstructure; // null in test mode

    /**
     * Teleop / auto constructor — also requires {@link Superstructure} so that any
     * running {@link ShootCommand} is interrupted and {@code handlePrepping()} stops
     * actively fighting this command over the turret angle.
     *
     * @param turret         The turret subsystem to home.
     * @param superstructure The superstructure state machine.
     */
    public HomeTurretCommand(TurretSubsystem turret, Superstructure superstructure) {
        m_turret         = turret;
        m_superstructure = superstructure;
        addRequirements(turret, superstructure);
    }

    /**
     * Test-mode constructor — requires only {@link TurretSubsystem}.
     * Use this variant in Test mode where {@link Superstructure#periodic()} is
     * already inactive and requiring Superstructure would be unnecessary.
     *
     * @param turret The turret subsystem to home.
     */
    public HomeTurretCommand(TurretSubsystem turret) {
        m_turret         = turret;
        m_superstructure = null;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Return to STOWED so handlePrepping() stops actively commanding the turret
        // to the hub angle every loop (which would fight this command).
        // Not needed in Test mode — Superstructure.periodic() is already inactive there.
        if (m_superstructure != null) {
            m_superstructure.requestState(RobotState.STOWED);
        }
        m_turret.setAngle(0.0);
    }

    @Override
    public void execute() {
        // Re-issue setAngle() every loop so the spring feedforward (computed from
        // current position inside setAngle) stays accurate throughout the move.
        // Phoenix 6 freezes the feedforward at the value in the last control request,
        // so without this the feedforward would be wrong the moment the turret starts moving.
        m_turret.setAngle(0.0);
    }

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
