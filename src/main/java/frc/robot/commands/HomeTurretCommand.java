package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurretSubsystem;

/**
 * Homes the turret to its mechanical zero position so that all subsequent
 * {@link TurretSubsystem#setAngle(double)} calls use a correct reference frame.
 *
 * <h2>Homing Strategies</h2>
 * <p>Supply a {@code limitSwitch} supplier at construction time to select one of
 * two homing strategies:
 *
 * <ol>
 *   <li><b>Limit-switch homing (preferred)</b> — When {@code limitSwitch} is
 *       non-null, the turret slowly rotates in the reverse direction until the
 *       supplier returns {@code true}, at which point the motor encoder is zeroed
 *       and the turret returns to 0°.  This is the most accurate method and should
 *       be used whenever a Hall-effect or contact limit switch is installed.</li>
 *   <li><b>Stall-detect homing (fallback)</b> — When {@code limitSwitch} is
 *       {@code null}, the turret slowly rotates in the reverse direction until the
 *       stator current exceeds {@link #STALL_CURRENT_THRESHOLD_A} for at least
 *       {@link #STALL_DURATION_S}.  The motor is then zeroed at the hard stop.
 *       <em>Use with caution</em> — this method contacts the physical stop and
 *       should only be used if no sensor is available.</li>
 * </ol>
 *
 * <p>After zeroing via either method, the turret returns to 0° (forward-facing).
 *
 * <p>Run this command once at the start of each match (e.g. bind to
 * {@code back + A} in {@link frc.robot.RobotContainer}).
 */
public class HomeTurretCommand extends Command {

    // -------------------------------------------------------------------------
    // Tuning Constants
    // -------------------------------------------------------------------------

    /** Open-loop duty cycle used during the slow homing rotation. */
    private static final double HOMING_DUTY_CYCLE = -0.08;

    /**
     * Stator current threshold (amps) above which the turret is considered to have
     * stalled against the hard stop.  Used only in the stall-detect strategy.
     */
    private static final double STALL_CURRENT_THRESHOLD_A = 15.0;

    /**
     * Duration (seconds) the stator current must exceed {@link #STALL_CURRENT_THRESHOLD_A}
     * before the hard-stop is confirmed.  Avoids false triggers from inertial spikes.
     */
    private static final double STALL_DURATION_S = 0.15;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    private final TurretSubsystem m_turret;
    private final BooleanSupplier m_limitSwitch; // null = stall-detect mode

    private final Timer m_stallTimer = new Timer();
    private boolean m_stallTimerRunning = false;
    private boolean m_homed = false;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * Constructs a HomeTurretCommand.
     *
     * @param turret      The turret subsystem to home.
     * @param limitSwitch Supplier that returns {@code true} when the turret is at
     *                    the home position (e.g. a Hall-effect or contact switch).
     *                    Pass {@code null} to use stall-detect homing instead.
     */
    public HomeTurretCommand(TurretSubsystem turret, BooleanSupplier limitSwitch) {
        m_turret      = turret;
        m_limitSwitch = limitSwitch;
        addRequirements(turret);
    }

    // -------------------------------------------------------------------------
    // Command Lifecycle
    // -------------------------------------------------------------------------

    @Override
    public void initialize() {
        m_homed            = false;
        m_stallTimerRunning = false;
        m_stallTimer.stop();
        m_stallTimer.reset();
    }

    @Override
    public void execute() {
        if (m_homed) {
            return;
        }

        if (m_limitSwitch != null) {
            // Strategy 1: limit-switch homing
            if (m_limitSwitch.getAsBoolean()) {
                zeroAndFinish();
                return;
            }
        } else {
            // Strategy 2: stall-detect homing
            double current = m_turret.getStatorCurrentAmps();
            if (current > STALL_CURRENT_THRESHOLD_A) {
                if (!m_stallTimerRunning) {
                    m_stallTimer.restart();
                    m_stallTimerRunning = true;
                } else if (m_stallTimer.hasElapsed(STALL_DURATION_S)) {
                    zeroAndFinish();
                    return;
                }
            } else {
                // Current dropped back below threshold — reset debounce timer.
                if (m_stallTimerRunning) {
                    m_stallTimer.stop();
                    m_stallTimer.reset();
                    m_stallTimerRunning = false;
                }
            }
        }

        // Continue rotating slowly toward the home position.
        m_turret.driveAtPercent(HOMING_DUTY_CYCLE);
    }

    @Override
    public boolean isFinished() {
        return m_homed;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
        // Always clean up the debounce timer so state does not leak into the
        // next invocation if the command is interrupted mid-homing.
        m_stallTimer.stop();
        m_stallTimer.reset();
        m_stallTimerRunning = false;
        if (!interrupted && m_homed) {
            // Move to confirmed zero position (forward-facing).
            m_turret.setAngle(0.0);
        }
    }

    // -------------------------------------------------------------------------
    // Private Helpers
    // -------------------------------------------------------------------------

    private void zeroAndFinish() {
        m_turret.stop();
        m_turret.zeroPosition();
        m_homed = true;
    }
}
