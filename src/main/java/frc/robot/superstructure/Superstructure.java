package frc.robot.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Feeder;
import frc.robot.Constants.Spindexer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.HubStateMonitor;
import frc.robot.util.HubStateMonitor.HubState;
import frc.robot.util.ShooterKinematics;
import frc.robot.util.ShooterKinematics.ShooterSetpoint;

/**
 * Central state machine governing the coordination of all scoring mechanisms.
 *
 * <p>The Superstructure owns the following robot states and drives the subsystems
 * accordingly each loop cycle:
 *
 * <ul>
 *   <li>{@link RobotState#STOWED} — All mechanisms at rest; intake retracted.</li>
 *   <li>{@link RobotState#INTAKING} — Intake deployed, spindexer running to accept balls.</li>
 *   <li>{@link RobotState#PREPPING_TO_SHOOT} — Turret tracking target; flywheel and hood
 *       spinning to calculated setpoints.  Does not fire.</li>
 *   <li>{@link RobotState#SHOOTING} — Continuous fire; feeder and spindexer run
 *       without interruption.  Transitions to EXHAUSTING if a jam is detected,
 *       then back to SHOOTING once cleared.</li>
 *   <li>{@link RobotState#EXHAUSTING} — Feeder and spindexer reverse briefly to clear
 *       a jammed ball, then return to the pre-jam state.</li>
 *   <li>{@link RobotState#TRAVERSING_TRENCH} — All mechanisms stowed for TRENCH transit.
 *       Only the drivetrain may operate; no scoring subsystems run.</li>
 * </ul>
 *
 * <h2>Continuous Fire</h2>
 * <p>Once the Superstructure enters SHOOTING (after the initial readiness gate in
 * {@link frc.robot.commands.ShootCommand}), the feeder and spindexer run
 * <em>without interruption</em> for the duration of the state.  The flywheel
 * setpoint is updated every loop by the active shoot command to track the
 * instantaneous effective distance; the flywheel adjusts while balls flow through.
 * No per-ball stop-and-wait cycle is used — the high-inertia steel flywheel wheels
 * maintain speed between shots and the control loop recovers during ball transit.
 *
 * <h2>Anti-Jam Logic</h2>
 * <p>While in SHOOTING or SHOOT_WHILE_INTAKING, the Superstructure monitors the
 * stator current of both the feeder and spindexer.  If either motor exceeds its
 * jam-detection threshold (configured in {@link Feeder} / {@link Spindexer}) for
 * longer than the configured jam duration, the Superstructure transitions to
 * EXHAUSTING, briefly reverses both motors, and then returns to the pre-jam state.
 *
 * <h2>Ball Counting</h2>
 * <p>The Superstructure tracks balls loaded via {@link #incrementBallCount()}.
 * Shot counting is not performed automatically in continuous-fire mode; install a
 * beam-break sensor at the shooter exit and call {@link #decrementBallCount()} from
 * its trigger for accurate tracking.  The count can be reset at match start via
 * {@link #resetBallCount()}.
 */
public class Superstructure extends SubsystemBase {

    // =========================================================================
    // State Enum
    // =========================================================================

    /** All possible high-level states of the robot's scoring superstructure. */
    public enum RobotState {
        /** All mechanisms idle; intake stowed. */
        STOWED,
        /** Intake deployed and running; spindexer spinning to accept balls. */
        INTAKING,
        /**
         * Turret locked on target; flywheel and hood at calculated setpoints.
         * Will not fire — awaiting readiness confirmation.
         */
        PREPPING_TO_SHOOT,
        /**
         * Continuous active firing: feeder and spindexer run without interruption.
         * The flywheel setpoint is updated every loop by the active shoot command.
         * Anti-jam monitoring is active; jams transition to EXHAUSTING and back.
         */
        SHOOTING,
        /**
         * Jam recovery: feeder and spindexer are reversed for a fixed duration,
         * then the Superstructure returns to the state that triggered exhaust.
         */
        EXHAUSTING,
        /**
         * TRENCH transit: all overhead mechanisms are stowed so the robot can pass
         * under the TRENCH structure.  Requested by
         * {@link frc.robot.subsystems.TrenchTraversalManager}.
         */
        TRAVERSING_TRENCH,
        /**
         * Simultaneous intake and shooting: intake is deployed and spindexer runs
         * continuously while the flywheel tracks the HUB.  The feeder only engages
         * once {@link ShooterSubsystem#isFlywheelAtSpeed()} returns {@code true},
         * avoiding wasted balls during spin-up.  Anti-jam logic mirrors
         * {@link RobotState#SHOOTING}.
         */
        SHOOT_WHILE_INTAKING
    }

    // =========================================================================
    // Subsystem References
    // =========================================================================

    private final ShooterSubsystem   m_shooter;
    private final TurretSubsystem    m_turret;
    private final FeederSubsystem    m_feeder;
    private final SpindexerSubsystem m_spindexer;
    private final IntakeSubsystem    m_intake;
    private final VisionSubsystem    m_vision;

    // =========================================================================
    // State Machine Variables
    // =========================================================================

    private RobotState m_state           = RobotState.STOWED;
    private RobotState m_preExhaustState = RobotState.STOWED;

    /**
     * Current shooter setpoint; updated each time distance changes.
     * Initialized at 4 m (mid-range) so anticipatory spin-up has a sensible default
     * before the first ShootCommand provides a vision-derived distance.
     */
    private ShooterSetpoint m_currentSetpoint = ShooterKinematics.calculate(4.0);

    // =========================================================================
    // Ball / Shot Counters
    // =========================================================================

    /**
     * Estimated number of balls currently held by the robot.  Incremented by
     * {@link #incrementBallCount()} when a ball is confirmed loaded (e.g. via
     * beam-break at the intake); decremented by {@link #decrementBallCount()}
     * when a shot is confirmed (e.g. via beam-break at the shooter exit).
     */
    private int m_ballCount = 0;

    /**
     * Latches {@code true} the first time {@link ShooterSubsystem#isFlywheelAtSpeed()}
     * returns {@code true} after entering {@link RobotState#SHOOT_WHILE_INTAKING}.
     * Once latched, the feeder runs continuously for the rest of that state —
     * no per-ball stop-and-wait.  Reset to {@code false} on every entry into
     * SHOOT_WHILE_INTAKING so the initial spin-up gate is always enforced.
     */
    private boolean m_feedingStarted = false;

    // =========================================================================
    // Timers
    // =========================================================================

    /** Measures how long the jam condition has been active continuously. */
    private final Timer m_jamTimer = new Timer();
    private boolean m_jamTimerRunning = false;

    /** Measures how long the exhaust cycle has been running. */
    private final Timer m_exhaustTimer = new Timer();

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Constructs the Superstructure with all required subsystem references.
     *
     * @param shooter    The flywheel and hood subsystem.
     * @param turret     The turret rotation subsystem.
     * @param feeder     The ball feeder subsystem.
     * @param spindexer  The spindexer disk subsystem.
     * @param intake     The ground intake subsystem.
     * @param vision     The vision subsystem (HUB targeting).
     */
    public Superstructure(
            ShooterSubsystem   shooter,
            TurretSubsystem    turret,
            FeederSubsystem    feeder,
            SpindexerSubsystem spindexer,
            IntakeSubsystem    intake,
            VisionSubsystem    vision) {
        m_shooter   = shooter;
        m_turret    = turret;
        m_feeder    = feeder;
        m_spindexer = spindexer;
        m_intake    = intake;
        m_vision    = vision;
    }

    // =========================================================================
    // Periodic — runs every robot loop
    // =========================================================================

    @Override
    public void periodic() {
        switch (m_state) {
            case STOWED                -> handleStowed();
            case INTAKING              -> handleIntaking();
            case PREPPING_TO_SHOOT     -> handlePrepping();
            case SHOOTING              -> handleShooting();
            case EXHAUSTING            -> handleExhausting();
            case TRAVERSING_TRENCH     -> handleTraversingTrench();
            case SHOOT_WHILE_INTAKING  -> handleShootWhileIntaking();
        }
        publishTelemetry();
    }

    // =========================================================================
    // State Request API (called by Commands)
    // =========================================================================

    /**
     * Requests a transition to the specified state.
     * The transition is applied on the next call to {@link #periodic()}, after any
     * guards defined in the handler have been evaluated.
     *
     * @param requestedState The desired new state.
     */
    public void requestState(RobotState requestedState) {
        if (requestedState == m_state) {
            return;
        }
        transitionTo(requestedState);
    }

    /**
     * Provides the Superstructure with updated shooter setpoints derived from the
     * current distance to the HUB.  Call this each loop from
     * {@link frc.robot.commands.ShootCommand} while in PREPPING_TO_SHOOT or SHOOTING.
     *
     * @param setpoint The setpoint computed by {@link ShooterKinematics#calculate(double)}.
     */
    public void applyShooterSetpoint(ShooterSetpoint setpoint) {
        m_currentSetpoint = setpoint;
        m_shooter.setFlywheelRPM(setpoint.flywheelRPM());
        m_shooter.setHoodAngle(setpoint.hoodAngleDeg());
    }

    /**
     * Commands the turret to a specific angle.  Intended to be called each loop
     * from {@link frc.robot.commands.ShootCommand} while vision is tracking a tag.
     *
     * @param angleDeg Target turret angle in degrees (0 = forward, positive = CCW).
     */
    public void commandTurretAngle(double angleDeg) {
        m_turret.setAngle(angleDeg);
    }

    // =========================================================================
    // Ball Count API
    // =========================================================================

    /**
     * Signals that one ball has been fired.  Call this from the beam-break trigger
     * on each shot (rising edge — ball clears the beam between feeder and flywheel).
     * The count is clamped to zero; it will not go negative.
     */
    public void decrementBallCount() {
        m_ballCount = Math.max(0, m_ballCount - 1);
    }

    /**
     * Presets the ball count to a known value.  Call at the start of each period
     * with the number of balls physically loaded into the robot (e.g. 3 for a
     * standard pre-load).  There is no intake sensor to count balls added during
     * play; only the shooter beam-break decrements this value.
     *
     * @param count Number of balls currently in the robot (0–5).
     */
    public void setBallCount(int count) {
        m_ballCount = Math.max(0, Math.min(count, 5));
    }

    /**
     * Returns the estimated number of balls currently held by the robot.
     * Starts at whatever was set by {@link #setBallCount(int)} and decrements
     * on each confirmed shot via the beam-break sensor.
     *
     * @return Ball count (0–5).
     */
    public int getBallCount() {
        return m_ballCount;
    }

    // =========================================================================
    // Readiness Queries
    // =========================================================================

    /**
     * Returns whether all three shot-readiness conditions are satisfied:
     * <ol>
     *   <li>Turret is aligned to its target.</li>
     *   <li>Flywheel is at its target speed.</li>
     *   <li>Hood is at its target angle.</li>
     * </ol>
     *
     * @return {@code true} if the robot is ready to fire.
     */
    public boolean isReadyToShoot() {
        return m_turret.isAligned()
            && m_shooter.isFlywheelAtSpeed()
            && m_shooter.isHoodAtAngle();
    }

    /**
     * Returns whether it is currently safe to transition mechanisms into a raised/extended
     * position (i.e. the robot is not traversing the TRENCH).
     *
     * @return {@code true} if mechanisms may extend freely.
     */
    public boolean isSafeToExtendMechanisms() {
        return m_state != RobotState.TRAVERSING_TRENCH;
    }

    /** @return The current superstructure state. */
    public RobotState getState() { return m_state; }

    /**
     * Returns the turret's current measured angle in degrees.
     * Exposed here so commands do not require a direct reference to
     * {@link frc.robot.subsystems.TurretSubsystem}.
     *
     * @return Turret angle in degrees (0 = forward, positive = CCW).
     */
    public double getTurretAngleDeg() { return m_turret.getAngleDeg(); }

    // =========================================================================
    // State Handlers
    // =========================================================================

    private void handleStowed() {
        m_intake.stow();
        m_shooter.stopFlywheel();
        m_shooter.stopHood();
        m_turret.stop();
        m_feeder.stop();
        m_spindexer.stop();
    }

    private void handleIntaking() {
        m_intake.deploy(); // deploys arm and runs roller
        m_shooter.stopFlywheel();
        m_shooter.stopHood();
        // Spindexer is intentionally NOT run during intaking — the intake roller
        // delivers the ball into the robot; the spindexer is only needed once the
        // ball is inside and we are ready to queue it for the feeder.
    }

    private void handlePrepping() {
        // Stow intake to avoid interference during the shot.
        m_intake.stow();
        // Shooter setpoints are applied externally via applyShooterSetpoint().
        // Spindexer runs slowly to keep balls queued without feeding.
        m_spindexer.run();
        m_feeder.stop();

        // Basic hub tracking: keep the turret locked on the hub regardless of
        // which direction the drivetrain faces.  Only a tx correction is applied
        // here — no moving-while-shooting lead angle.
        // ShootCommand.execute() runs after periodic() each loop, so when it is
        // active it transparently overrides this with the full compensation
        // (radial d_eff + lateral lead angle).
        m_vision.getTargetTxDeg().ifPresent(tx -> {
            // tx > 0  →  tag is right of camera crosshair  →  rotate turret CW (negative)
            m_turret.setAngle(m_turret.getAngleDeg() - tx);
        });
    }

    private void handleShooting() {
        // Abort if the HUB has gone inactive.
        if (HubStateMonitor.getHubState() == HubState.INACTIVE) {
            transitionTo(RobotState.PREPPING_TO_SHOOT);
            return;
        }

        // Continuous fire: feeder and spindexer run without interruption.
        // The flywheel setpoint is updated every loop by the active shoot command;
        // the flywheel adjusts while balls flow through.  The high-inertia steel
        // flywheel wheels maintain sufficient speed between consecutive shots so
        // no per-ball stop-and-wait cycle is necessary.
        m_feeder.feed();
        m_spindexer.run();

        // Anti-jam: monitor stator current on feeder and spindexer.
        boolean jamDetected = m_feeder.getStatorCurrentAmps()    > Feeder.FEEDER_JAM_CURRENT_A
                           || m_spindexer.getStatorCurrentAmps() > Spindexer.SPINDEXER_JAM_CURRENT_A;

        if (jamDetected) {
            if (!m_jamTimerRunning) {
                m_jamTimer.restart();
                m_jamTimerRunning = true;
            } else if (m_jamTimer.hasElapsed(Feeder.FEEDER_JAM_DURATION_S)) {
                m_preExhaustState = m_state;
                m_jamTimer.stop();
                m_jamTimerRunning = false;
                transitionTo(RobotState.EXHAUSTING);
            }
        } else {
            if (m_jamTimerRunning) {
                m_jamTimer.stop();
                m_jamTimerRunning = false;
            }
        }
    }

    private void handleExhausting() {
        m_feeder.reverse();
        m_spindexer.reverse();

        if (m_exhaustTimer.hasElapsed(Feeder.FEEDER_EXHAUST_DURATION_S)) {
            transitionTo(m_preExhaustState);
        }
    }

    private void handleShootWhileIntaking() {
        if (HubStateMonitor.getHubState() == HubState.INACTIVE) {
            transitionTo(RobotState.STOWED);
            return;
        }

        m_intake.deploy();   // deploy arm + run roller
        m_spindexer.run();   // continuously queue balls

        // Gate the feeder on the first moment the flywheel reaches speed (avoids
        // wasting balls during the initial spin-up).  Once the latch trips, the
        // feeder runs continuously for the rest of this state — no per-ball
        // stop-and-wait, same as SHOOTING.
        if (!m_feedingStarted && m_shooter.isFlywheelAtSpeed()) {
            m_feedingStarted = true;
        }
        if (m_feedingStarted) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }

        // Anti-jam: same logic as SHOOTING state (reuses m_jamTimer / m_jamTimerRunning).
        boolean jamDetected = m_feeder.getStatorCurrentAmps()    > Feeder.FEEDER_JAM_CURRENT_A
                           || m_spindexer.getStatorCurrentAmps() > Spindexer.SPINDEXER_JAM_CURRENT_A;
        if (jamDetected) {
            if (!m_jamTimerRunning) {
                m_jamTimer.restart();
                m_jamTimerRunning = true;
            } else if (m_jamTimer.hasElapsed(Feeder.FEEDER_JAM_DURATION_S)) {
                m_preExhaustState = m_state;
                m_jamTimer.stop();
                m_jamTimerRunning = false;
                transitionTo(RobotState.EXHAUSTING);
            }
        } else {
            if (m_jamTimerRunning) {
                m_jamTimer.stop();
                m_jamTimerRunning = false;
            }
        }
    }

    private void handleTraversingTrench() {
        // All overhead mechanisms must be stowed for TRENCH clearance.
        m_intake.stow();
        m_shooter.stopFlywheel();
        m_shooter.stopHood();
        m_turret.stop();
        m_feeder.stop();
        m_spindexer.stop();
    }

    // =========================================================================
    // Telemetry
    // =========================================================================

    private void publishTelemetry() {
        SmartDashboard.putString("Superstructure/State",         m_state.name());
        SmartDashboard.putBoolean("Superstructure/ReadyToShoot", isReadyToShoot());
        SmartDashboard.putNumber("Superstructure/BallCount",     m_ballCount);
        SmartDashboard.putNumber("Superstructure/FlywheelRPM",   m_shooter.getFlywheelRPM());
        SmartDashboard.putNumber("Superstructure/TargetRPM",     m_shooter.getTargetFlywheelRPM());
        SmartDashboard.putNumber("Superstructure/HoodAngleDeg",  m_shooter.getHoodAngleDeg());
        SmartDashboard.putNumber("Superstructure/TargetHoodDeg", m_shooter.getTargetHoodAngleDeg());
        SmartDashboard.putNumber("Superstructure/TurretAngleDeg",m_turret.getAngleDeg());
        SmartDashboard.putNumber("Superstructure/TargetTurretDeg",m_turret.getTargetAngleDeg());
        SmartDashboard.putBoolean("Superstructure/FlywheelAtSpeed", m_shooter.isFlywheelAtSpeed());
        SmartDashboard.putBoolean("Superstructure/HoodAtAngle",     m_shooter.isHoodAtAngle());
        SmartDashboard.putBoolean("Superstructure/TurretAligned",   m_turret.isAligned());
    }

    /**
     * Executes all entry actions for the given state and updates the current-state field.
     *
     * @param newState The state to transition into.
     */
    private void transitionTo(RobotState newState) {
        switch (newState) {
            case STOWED -> {
                // All subsystems stop; handled in handleStowed() next loop.
            }
            case INTAKING -> {
                // Nothing additional on entry; intake starts in handleIntaking().
            }
            case PREPPING_TO_SHOOT -> {
                m_feeder.stop();
            }
            case SHOOTING -> {
                m_jamTimerRunning = false;
                m_jamTimer.stop();
                m_jamTimer.reset();
            }
            case EXHAUSTING -> {
                m_exhaustTimer.restart();
                // handleExhausting() runs reverse() every loop — no need to duplicate here.
            }
            case TRAVERSING_TRENCH -> {
                // Immediately stop all scoring mechanisms on TRENCH entry.
                m_feeder.stop();
                m_spindexer.stop();
                m_shooter.stopFlywheel();
                m_shooter.stopHood();
                m_turret.stop();
                m_intake.stow();
            }
            case SHOOT_WHILE_INTAKING -> {
                m_feedingStarted  = false; // re-arm the spin-up gate for this entry
                m_jamTimerRunning = false;
                m_jamTimer.stop();
                m_jamTimer.reset();
            }
        }

        m_state = newState;
    }
}
