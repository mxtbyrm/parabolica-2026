package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants.SuperstructureConstants;

/**
 * Determines whether the robot's alliance HUB is currently active (eligible to
 * score FUEL) using the official 2026 FMS game-specific message and match time.
 *
 * <h2>HUB Activity Rule (2026 REBUILT)</h2>
 * <p>The alliance that scored more FUEL in Auto goes inactive first.  The FMS
 * transmits a single-character game-data message approximately 3 s after Auto ends:
 * <ul>
 *   <li>{@code 'R'} — Red alliance's HUB goes inactive first.</li>
 *   <li>{@code 'B'} — Blue alliance's HUB goes inactive first.</li>
 * </ul>
 * <p>The HUB alternates between active and inactive in 25-second <em>shifts</em>
 * during the 150-second Teleop period.  The alliance whose HUB goes inactive first
 * is active in <b>Shifts 2 and 4</b>; the other alliance is active in
 * <b>Shifts 1 and 3</b>.
 *
 * <h2>Shift Timing (seconds remaining in Teleop)</h2>
 * <pre>
 *   t > 130   Opening transition  — both HUBs always active
 *   130 ≥ t > 105   Shift 1  (25 s)
 *   105 ≥ t >  80   Shift 2  (25 s)
 *    80 ≥ t >  55   Shift 3  (25 s)
 *    55 ≥ t >  30   Shift 4  (25 s)
 *    t ≤  30        End Game — both HUBs always active
 * </pre>
 *
 * <h2>PULSING (deactivation warning)</h2>
 * <p>During the 3 seconds immediately before a shift change that deactivates this
 * alliance's HUB, {@link #getHubState()} returns {@link HubState#PULSING}.
 * New shots are inhibited within {@link SuperstructureConstants#HUB_PULSE_SAFETY_MARGIN_S}
 * of the deactivation point to ensure in-flight balls arrive before the HUB
 * goes dark.
 *
 * <h2>Special Cases</h2>
 * <ul>
 *   <li><b>Autonomous</b> — HUB always active.</li>
 *   <li><b>No game data yet</b> (first ~3 s of Teleop) — assume active.</li>
 *   <li><b>No FMS / no alliance</b> — return UNKNOWN; treat as not eligible.</li>
 *   <li><b>Disabled / Test</b> — return INACTIVE.</li>
 * </ul>
 */
public final class HubStateMonitor {

    /** Duration of the HUB pulsing (deactivation-warning) window in seconds. */
    public static final double PULSE_WINDOW_S = 3.0;

    /**
     * Match-time thresholds (seconds remaining) marking the end of each
     * active shift.  A HUB that is active in the shift just above each
     * threshold will go inactive as the counter crosses it.
     */
    private static final double[] SHIFT_BOUNDARIES_S = {105.0, 80.0, 55.0, 30.0};

    private HubStateMonitor() {} // Utility class — do not instantiate.

    // =========================================================================
    // Public API
    // =========================================================================

    /**
     * The possible states of the HUB lighting and scoring eligibility.
     */
    public enum HubState {
        /** HUB is active; solid alliance color; shots score normally. */
        ACTIVE,
        /**
         * HUB is in the {@link #PULSE_WINDOW_S}-second deactivation-warning window;
         * pulsing alliance color.  Shots fired during PULSING may still score if
         * the ball enters before the shift changes — see {@link #isSafeToBeginShot()}.
         */
        PULSING,
        /** HUB is inactive; lights off; shots do not score. */
        INACTIVE,
        /** State cannot be determined (no FMS connection or unrecognised data). */
        UNKNOWN
    }

    /**
     * Returns the current HUB state for the robot's alliance.
     *
     * @return The resolved {@link HubState}; never {@code null}.
     */
    public static HubState getHubState() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return HubState.UNKNOWN;
        }

        // Autonomous — both HUBs always active.
        if (DriverStation.isAutonomousEnabled()) {
            return HubState.ACTIVE;
        }

        // Not in Teleop → no hub activity (disabled, test, etc.).
        if (!DriverStation.isTeleopEnabled()) {
            return HubState.INACTIVE;
        }

        // --- Teleop: compute from match time and game data ---

        double matchTime = DriverStation.getMatchTime();
        String gameData  = DriverStation.getGameSpecificMessage();

        // No game data yet (sent ~3 s after Auto ends) — assume active.
        if (gameData == null || gameData.isEmpty()) {
            return HubState.ACTIVE;
        }

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default  -> { return HubState.ACTIVE; } // Corrupt data — default safe.
        }

        // shift1Active: is THIS alliance's HUB active during Shifts 1 and 3?
        //   Red active in Shift 1 if Red is NOT the one going inactive first.
        //   Blue active in Shift 1 if Red IS the one going inactive first.
        boolean shift1Active = switch (alliance.get()) {
            case Red  -> !redInactiveFirst;
            case Blue ->  redInactiveFirst;
        };

        return computeShiftState(matchTime, shift1Active);
    }

    /**
     * Returns {@code true} if the HUB is currently eligible to score FUEL
     * (states {@link HubState#ACTIVE} and {@link HubState#PULSING}).
     *
     * @return {@code true} if shooting is scoring-eligible.
     */
    public static boolean isHubScoringEligible() {
        HubState state = getHubState();
        return state == HubState.ACTIVE || state == HubState.PULSING;
    }

    /**
     * Returns {@code true} if it is safe to begin a new shot that will score.
     *
     * <p>During the {@link HubState#PULSING} window, new shots are only allowed if
     * there is more than {@link SuperstructureConstants#HUB_PULSE_SAFETY_MARGIN_S}
     * of time remaining before the HUB deactivates.  This prevents balls from
     * being in flight when the shift changes.
     *
     * @return {@code true} if a new shot can safely start.
     */
    public static boolean isSafeToBeginShot() {
        HubState state = getHubState();
        if (state == HubState.ACTIVE) {
            return true;
        }
        if (state != HubState.PULSING) {
            return false;
        }

        // In PULSING: only allow shots if there is more than the safety margin
        // before the upcoming deactivation boundary.
        double matchTime = DriverStation.getMatchTime();
        for (double boundary : SHIFT_BOUNDARIES_S) {
            if (matchTime > boundary && matchTime <= boundary + PULSE_WINDOW_S) {
                double timeUntilBoundary = matchTime - boundary;
                return timeUntilBoundary > SuperstructureConstants.HUB_PULSE_SAFETY_MARGIN_S;
            }
        }
        // Fallback — should not be reached while in PULSING.
        return true;
    }

    // =========================================================================
    // Private Helpers
    // =========================================================================

    /**
     * Maps {@code matchTime} (seconds remaining in Teleop) to the appropriate
     * {@link HubState} for an alliance that is active in Shifts 1 and 3 when
     * {@code shift1Active} is {@code true}.
     */
    private static HubState computeShiftState(double matchTime, boolean shift1Active) {
        if (matchTime > 130) {
            // Opening transition window — both HUBs active.
            return HubState.ACTIVE;
        } else if (matchTime > 105) {
            // Shift 1
            return stateForShift(shift1Active, matchTime, 105.0);
        } else if (matchTime > 80) {
            // Shift 2
            return stateForShift(!shift1Active, matchTime, 80.0);
        } else if (matchTime > 55) {
            // Shift 3
            return stateForShift(shift1Active, matchTime, 55.0);
        } else if (matchTime > 30) {
            // Shift 4
            return stateForShift(!shift1Active, matchTime, 30.0);
        } else {
            // End game — both HUBs always active.
            return HubState.ACTIVE;
        }
    }

    /**
     * Returns the {@link HubState} for a single shift.
     *
     * @param hubActiveThisShift Whether this alliance's HUB is active during this shift.
     * @param matchTime          Current match time (seconds remaining).
     * @param shiftEndBoundary   The match-time value at which this shift ends and the
     *                           next shift begins (also the deactivation point for an
     *                           active-in-this-shift hub).
     */
    private static HubState stateForShift(boolean hubActiveThisShift,
                                           double  matchTime,
                                           double  shiftEndBoundary) {
        if (!hubActiveThisShift) {
            return HubState.INACTIVE;
        }
        // Active — check if we are entering the pulsing (deactivation-warning) window.
        if (matchTime <= shiftEndBoundary + PULSE_WINDOW_S) {
            return HubState.PULSING;
        }
        return HubState.ACTIVE;
    }
}
