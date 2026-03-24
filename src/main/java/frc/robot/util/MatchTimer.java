package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Publishes a single global match-phase countdown to SmartDashboard.
 *
 * <h2>Match timeline</h2>
 * <pre>
 *   Auto          20 s   — both HUBs active
 *   Transition    10 s   — both HUBs active  (t > 130 in Teleop)
 *   Shift 1       25 s   — auto-winner INACTIVE  (130 ≥ t > 105)
 *   Shift 2       25 s   — auto-winner ACTIVE    (105 ≥ t >  80)
 *   Shift 3       25 s   — auto-winner INACTIVE   (80 ≥ t >  55)
 *   Shift 4       25 s   — auto-winner ACTIVE      (55 ≥ t >  30)
 *   End Game      30 s   — both HUBs active         (t ≤  30)
 * </pre>
 *
 * <p>The "auto-winner" alliance is whoever scored more FUEL in Auto; the FMS
 * sends {@code 'R'} or {@code 'B'} in the game-specific message ~3 s after Auto.
 * That alliance starts as INACTIVE in Shift 1.  Before the message arrives the
 * hub columns show {@code "?"}.
 *
 * <h2>SmartDashboard keys</h2>
 * <ul>
 *   <li>{@code Match/Phase}         — "Auto" | "Transition" | "Shift 1" … "Shift 4" | "End Game"</li>
 *   <li>{@code Match/PhaseCountdown}— "0:23" — seconds left in the CURRENT phase</li>
 *   <li>{@code Match/RedHub}        — "ACTIVE" | "INACTIVE" | "?"</li>
 *   <li>{@code Match/BlueHub}       — "ACTIVE" | "INACTIVE" | "?"</li>
 *   <li>{@code Match/FMSConnected}  — boolean</li>
 * </ul>
 *
 * <p>Works without FMS: uses an internal {@link Timer} started at each period
 * enable.  Auto = 20 s, Teleop = 140 s.
 */
public class MatchTimer {

    // Official 2026 durations used for the internal fallback (no FMS).
    private static final double AUTO_DURATION_S   = 20.0;
    private static final double TELEOP_DURATION_S = 140.0; // 10 + 25*4 + 30

    // Teleop shift boundaries (seconds remaining in Teleop).
    private static final double T_TRANSITION_END = 130.0; // end of 10-s transition
    private static final double T_SHIFT1_END     = 105.0;
    private static final double T_SHIFT2_END     =  80.0;
    private static final double T_SHIFT3_END     =  55.0;
    private static final double T_SHIFT4_END     =  30.0; // end game starts here

    private final Timer  m_timer          = new Timer();
    private double       m_periodDuration = 0.0;

    // ── Period lifecycle ──────────────────────────────────────────────────────

    /** Call at the top of every {@code *Init()} method in {@code Robot}. */
    public void onPeriodStart() {
        m_timer.restart();
        if      (DriverStation.isAutonomous()) m_periodDuration = AUTO_DURATION_S;
        else if (DriverStation.isTeleop())     m_periodDuration = TELEOP_DURATION_S;
        else                                   m_periodDuration = 0.0;
    }

    // ── Main publish ──────────────────────────────────────────────────────────

    /** Call every robot loop from {@code robotPeriodic()}. */
    public void periodic() {
        boolean fms     = DriverStation.isFMSAttached();
        double  fmsTime = DriverStation.getMatchTime(); // -1 if no FMS
        double  elapsed = m_timer.get();

        // Seconds remaining in the CURRENT period (auto or teleop).
        double timeLeft = (fms && fmsTime >= 0)
                ? fmsTime
                : Math.max(0.0, m_periodDuration - elapsed);

        // ── Resolve phase ─────────────────────────────────────────────────────
        String phase;
        double phaseTimeLeft;

        if (DriverStation.isAutonomous()) {
            phase         = "Auto";
            phaseTimeLeft = timeLeft;

        } else if (DriverStation.isTeleop()) {
            if (timeLeft > T_TRANSITION_END) {
                phase         = "Transition";
                phaseTimeLeft = timeLeft - T_TRANSITION_END;
            } else if (timeLeft > T_SHIFT1_END) {
                phase         = "Shift 1";
                phaseTimeLeft = timeLeft - T_SHIFT1_END;
            } else if (timeLeft > T_SHIFT2_END) {
                phase         = "Shift 2";
                phaseTimeLeft = timeLeft - T_SHIFT2_END;
            } else if (timeLeft > T_SHIFT3_END) {
                phase         = "Shift 3";
                phaseTimeLeft = timeLeft - T_SHIFT3_END;
            } else if (timeLeft > T_SHIFT4_END) {
                phase         = "Shift 4";
                phaseTimeLeft = timeLeft - T_SHIFT4_END;
            } else {
                phase         = "End Game";
                phaseTimeLeft = timeLeft;
            }

        } else if (DriverStation.isDisabled()) {
            phase         = "Disabled";
            phaseTimeLeft = 0;
        } else {
            phase         = "Test";
            phaseTimeLeft = 0;
        }

        // ── Hub states ────────────────────────────────────────────────────────
        // FMS game data: 'R' = Red auto-winner (inactive first),
        //                'B' = Blue auto-winner (inactive first).
        String gameData = DriverStation.getGameSpecificMessage();
        boolean hasData = gameData != null && !gameData.isEmpty();
        boolean redInactiveFirst  = hasData && gameData.charAt(0) == 'R';
        boolean blueInactiveFirst = hasData && gameData.charAt(0) == 'B';

        String redHub  = hubForPhase(phase, redInactiveFirst,  hasData);
        String blueHub = hubForPhase(phase, blueInactiveFirst, hasData);

        // ── Publish ───────────────────────────────────────────────────────────
        SmartDashboard.putString ("Match/Phase",          phase);
        SmartDashboard.putString ("Match/PhaseCountdown", formatTime(phaseTimeLeft));
        SmartDashboard.putNumber ("Match/PhaseSeconds",   phaseTimeLeft);
        SmartDashboard.putString ("Match/RedHub",         redHub);
        SmartDashboard.putString ("Match/BlueHub",        blueHub);
        SmartDashboard.putBoolean("Match/FMSConnected",   fms);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * Hub activity for a given phase.
     *
     * @param phase           Current phase name.
     * @param inactiveFirst   True if this hub's alliance is the one going inactive first.
     * @param hasData         True if the FMS game-specific message has been received.
     */
    private static String hubForPhase(String phase, boolean inactiveFirst, boolean hasData) {
        return switch (phase) {
            // Both always active
            case "Auto", "Transition", "End Game", "Disabled", "Test" -> "ACTIVE";
            // Auto-winner (inactiveFirst) is INACTIVE in odd shifts, ACTIVE in even shifts
            case "Shift 1", "Shift 3" -> hasData ? (inactiveFirst ? "INACTIVE" : "ACTIVE") : "?";
            case "Shift 2", "Shift 4" -> hasData ? (inactiveFirst ? "ACTIVE" : "INACTIVE") : "?";
            default -> "?";
        };
    }

    /** Formats seconds as {@code "M:SS"}, rounding up to the next whole second. */
    private static String formatTime(double seconds) {
        int s = Math.max(0, (int) Math.ceil(seconds));
        return String.format("%d:%02d", s / 60, s % 60);
    }
}
