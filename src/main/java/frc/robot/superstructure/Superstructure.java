package frc.robot.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Feeder;
import frc.robot.Constants.Spindexer;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.HubStateMonitor;
import frc.robot.util.HubStateMonitor.HubState;
import frc.robot.util.ShooterKinematics.ShooterSetpoint;

/**
 * Central state machine governing the coordination of all scoring mechanisms.
 *
 * <p>The Superstructure owns the following robot states and drives the subsystems
 * accordingly each loop cycle:
 *
 * <ul>
 *   <li>{@link RobotState#STOWED} — All mechanisms at rest; intake roller stopped
 *       (arm position left to operator).</li>
 *   <li>{@link RobotState#INTAKING} — Intake deployed and running; feeder and
 *       spindexer stopped so balls settle cleanly into the magazine.</li>
 *   <li>{@link RobotState#PREPPING_TO_SHOOT} — Turret tracking HUB; flywheel and
 *       hood at calculated setpoints; spindexer keeps balls queued.  Does not fire.</li>
 *   <li>{@link RobotState#SHOOTING} — Continuous fire; feeder and spindexer run
 *       without interruption.  Anti-jam monitoring active.</li>
 *   <li>{@link RobotState#SHOOT_WHILE_INTAKING} — Intake deployed simultaneously
 *       with hub tracking and continuous fire.  Feeder gated on flywheel reaching
 *       speed.  Anti-jam monitoring active.</li>
 *   <li>{@link RobotState#PASSING_TO_ALLIANCE} — Inactive-period pass: flywheel/hood
 *       at fixed pass setpoints; feeder and spindexer lob balls to the alliance zone;
 *       intake arm auto-stowed.  Anti-jam monitoring active.</li>
 *   <li>{@link RobotState#EXHAUSTING} — Feeder and spindexer reverse briefly to
 *       clear a jammed ball, then return to the pre-jam state.</li>
 *   <li>{@link RobotState#TRAVERSING_TRENCH} — All scoring mechanisms stopped;
 *       intake arm is operator-controlled.  {@link #applyShooterSetpoint} and
 *       {@link #commandTurretAngle} are no-ops in this state.</li>
 * </ul>
 *
 * <h2>Continuous Fire</h2>
 * <p>Once in SHOOTING, the feeder and spindexer run <em>without interruption</em>.
 * The flywheel setpoint is updated every loop by the active shoot command; the
 * high-inertia steel flywheel wheels maintain speed between shots and the control
 * loop recovers during ball transit.  No per-ball stop-and-wait cycle is used.
 *
 * <h2>Anti-Jam Logic</h2>
 * <p>While in SHOOTING, SHOOT_WHILE_INTAKING, or PASSING_TO_ALLIANCE, the
 * Superstructure monitors stator current on both the feeder and spindexer via
 * {@link #applyJamDetection()}.  If either motor exceeds its jam-detection
 * threshold for longer than {@link Feeder#FEEDER_JAM_DURATION_S}, the machine
 * transitions to EXHAUSTING, briefly reverses both motors, then returns to the
 * pre-jam state.
 *
 * <h2>TRAVERSING_TRENCH Guard</h2>
 * <p>{@link #applyShooterSetpoint} and {@link #commandTurretAngle} are no-ops
 * while in TRAVERSING_TRENCH, preventing any concurrently running shoot command
 * from re-energizing the shooter while the robot is inside the TRENCH.
 *
 * <h2>Ball Counting</h2>
 * <p>Ball count is preset at match start via {@link #setBallCount(int)} (called
 * from {@link frc.robot.RobotContainer#prepareForMatch()}).  Each confirmed shot
 * decrements the count via {@link #decrementBallCount()}, triggered by the
 * CANrange sensor between the feeder exit and the flywheel.  There is no intake
 * sensor — balls collected during play are not counted automatically.
 */
public class Superstructure extends SubsystemBase {

    // =========================================================================
    // State Enum
    // =========================================================================

    /** All possible high-level states of the robot's scoring superstructure. */
    public enum RobotState {
        /** All mechanisms idle; intake roller stopped (arm position left to operator). */
        STOWED,

        /**
         * Intake deployed and running; feeder and spindexer explicitly stopped
         * so new balls settle into the magazine without being pre-fed.
         */
        INTAKING,

        /**
         * Turret locked on target; flywheel and hood at calculated setpoints;
         * spindexer keeps balls queued.  Will not fire — awaiting readiness.
         */
        PREPPING_TO_SHOOT,

        /**
         * Continuous active firing: feeder and spindexer run without interruption.
         * Flywheel setpoint updated every loop by the active shoot command.
         * Anti-jam monitoring active; jams transition to EXHAUSTING and back.
         */
        SHOOTING,

        /**
         * Jam recovery: feeder and spindexer reversed for a fixed duration,
         * then the machine returns to the state that triggered exhaust.
         */
        EXHAUSTING,

        /**
         * TRENCH transit: all scoring mechanisms stopped; intake arm position
         * NOT changed (operator explicit control).  {@link #applyShooterSetpoint}
         * and {@link #commandTurretAngle} are no-ops here.
         * Requested by {@link frc.robot.subsystems.TrenchTraversalManager}.
         */
        TRAVERSING_TRENCH,

        /**
         * Simultaneous intake and shooting: intake deployed and spindexer running
         * while the flywheel tracks the HUB.  Feeder only engages once
         * {@link ShooterSubsystem#isFlywheelAtSpeed()} is true, avoiding wasted
         * balls during spin-up.  Anti-jam monitoring active.
         */
        SHOOT_WHILE_INTAKING,

        /**
         * Inactive-period pass: turret faces alliance wall (commanded by the active
         * shoot command), hood and flywheel at fixed pass setpoints, feeder/spindexer
         * lob balls to the alliance zone.  Intake arm auto-stowed.
         * Entered automatically from SHOOTING or SHOOT_WHILE_INTAKING when
         * {@link HubState#INACTIVE} is detected.  Exits automatically back to
         * PREPPING_TO_SHOOT when the active period resumes.
         */
        PASSING_TO_ALLIANCE
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

    // =========================================================================
    // Ball Counter
    // =========================================================================

    /**
     * Estimated number of balls currently held by the robot.  Set at match start
     * via {@link #setBallCount(int)}; decremented by {@link #decrementBallCount()}
     * on each confirmed shot (CANrange sensor at shooter exit).
     * There is no intake sensor — balls collected during play are not counted.
     */
    private int m_ballCount = 0;

    /**
     * Latches {@code true} the first time {@link ShooterSubsystem#isFlywheelAtSpeed()}
     * returns {@code true} after entering {@link RobotState#SHOOT_WHILE_INTAKING}.
     * Once latched, the feeder runs continuously — same as SHOOTING.
     * Reset to {@code false} on every entry into SHOOT_WHILE_INTAKING so the
     * initial spin-up gate is enforced on each new activation.
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
        // Always publish telemetry — even in Test mode — so SmartDashboard reflects
        // live subsystem state while the operator drives mechanisms directly.
        publishTelemetry();

        // In Test mode the operator drives subsystems directly via the controller
        // bindings defined in RobotContainer#configureTestBindings().  The state
        // machine must not run concurrently or it will fight those direct commands.
        if (DriverStation.isTest()) return;

        switch (m_state) {
            case STOWED                -> handleStowed();
            case INTAKING              -> handleIntaking();
            case PREPPING_TO_SHOOT     -> handlePrepping();
            case SHOOTING              -> handleShooting();
            case EXHAUSTING            -> handleExhausting();
            case TRAVERSING_TRENCH     -> handleTraversingTrench();
            case SHOOT_WHILE_INTAKING  -> handleShootWhileIntaking();
            case PASSING_TO_ALLIANCE   -> handlePassingToAlliance();
        }
    }

    // =========================================================================
    // State Request API (called by Commands)
    // =========================================================================

    /**
     * Requests a transition to the specified state.  Entry actions in
     * {@link #transitionTo} execute immediately; the new handler takes effect
     * on the next {@link #periodic()} call.
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
     * current distance to the HUB.  Call each loop from
     * {@link frc.robot.commands.ShootCommand} while in PREPPING_TO_SHOOT or SHOOTING.
     *
     * <p><b>No-op while in {@link RobotState#TRAVERSING_TRENCH}</b> — prevents any
     * concurrently running shoot command from re-energizing the shooter while the
     * robot is inside the TRENCH.
     *
     * @param setpoint The setpoint computed by {@link ShooterKinematics#calculate}.
     */
    public void applyShooterSetpoint(ShooterSetpoint setpoint) {
        if (m_state == RobotState.TRAVERSING_TRENCH) return;
        m_shooter.setFlywheelRPM(setpoint.flywheelRPM());
        m_shooter.setHoodAngle(setpoint.hoodAngleDeg());
    }

    /**
     * Commands the turret to a specific angle.  Intended to be called each loop
     * from {@link frc.robot.commands.ShootCommand} while vision is tracking a tag.
     *
     * <p><b>No-op while in {@link RobotState#TRAVERSING_TRENCH}.</b>
     *
     * @param angleDeg Target turret angle in degrees (0 = forward, positive = CCW).
     */
    public void commandTurretAngle(double angleDeg) {
        if (m_state == RobotState.TRAVERSING_TRENCH) return;
        m_turret.setAngle(angleDeg);
    }

    // =========================================================================
    // Ball Count API
    // =========================================================================

    /**
     * Signals that one ball has been fired.  Call this from the CANrange trigger
     * on each confirmed shot (ball fully clears the sensor between feeder and
     * flywheel).  The count is clamped to zero; it will not go negative.
     */
    public void decrementBallCount() {
        m_ballCount = Math.max(0, m_ballCount - 1);
    }

    /**
     * Presets the ball count to a known value.  Call at the start of each period
     * with the number of balls physically loaded into the robot.  There is no
     * intake sensor to count balls added during play; only the shooter CANrange
     * decrements this value.
     *
     * @param count Number of balls currently in the robot (0–5).
     */
    public void setBallCount(int count) {
        m_ballCount = Math.max(0, Math.min(count, 5));
    }

    /**
     * Returns the estimated number of balls currently held by the robot.
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
     * turret aligned, flywheel at speed, and hood at angle.
     *
     * @return {@code true} if the robot is ready to fire.
     */
    public boolean isReadyToShoot() {
        return m_turret.isAligned()
            && m_shooter.isFlywheelAtSpeed()
            && m_shooter.isHoodAtAngle();
    }

    /**
     * Returns whether it is safe to extend mechanisms (i.e. not traversing TRENCH).
     *
     * @return {@code true} if mechanisms may extend freely.
     */
    public boolean isSafeToExtendMechanisms() {
        return m_state != RobotState.TRAVERSING_TRENCH;
    }

    /** @return The current superstructure state. */
    public RobotState getState() { return m_state; }

    /**
     * Returns the turret's current measured angle in degrees so commands do not
     * need a direct reference to {@link TurretSubsystem}.
     *
     * @return Turret angle in degrees (0 = forward, positive = CCW).
     */
    public double getTurretAngleDeg() { return m_turret.getAngleDeg(); }

    // =========================================================================
    // State Handlers
    // =========================================================================

    private void handleStowed() {
        m_intake.stopRoller();    // arm position left to operator — only roller stopped
        m_shooter.stopFlywheel();
        m_shooter.stopHood();
        m_turret.stop();
        m_feeder.stop();
        m_spindexer.stop();
    }

    private void handleIntaking() {
        m_intake.deploy();        // move arm to deployed position
        m_intake.runRoller();     // run roller only once arm is deployed (guard in IntakeSubsystem)
        m_feeder.stop();          // must stay off — ball arrives through intake, not feeder
        m_spindexer.stop();       // not needed during intake; roller delivers ball into magazine
        m_shooter.stopFlywheel();
        m_shooter.stopHood();
        // Turret holds its last commanded position via brake mode.
        // No active tracking while intaking — operator focus is on field coverage.
    }

    private void handlePrepping() {
        m_intake.stopRoller();    // stop roller; arm position is operator-controlled
        m_spindexer.run();        // keep balls queued toward feeder without firing
        m_feeder.stop();

        // Basic hub tracking.  ShootCommand.execute() runs AFTER periodic() each loop
        // and overrides this with full moving-while-shooting compensation (radial d_eff
        // + lateral lead angle + turret pivot offset) when it is active.  This handler
        // provides continuous turret pointing when no shoot command is running.
        m_vision.getTargetTxDeg().ifPresentOrElse(
            tx -> m_turret.setAngle(m_turret.getAngleDeg() - tx),
            ()  -> m_vision.getHubRobotRelativeAngleDeg().ifPresent(m_turret::setAngle)
        );
    }

    private void handleShooting() {
        // Inactive period starts: switch to alliance-wall pass automatically.
        if (HubStateMonitor.getHubState() == HubState.INACTIVE) {
            transitionTo(RobotState.PASSING_TO_ALLIANCE);
            return;
        }

        m_intake.stopRoller();    // no intaking during pure-shooting mode
        m_feeder.feed();
        m_spindexer.run();
        applyJamDetection();
    }

    private void handleExhausting() {
        m_feeder.reverse();
        m_spindexer.reverse();
        if (m_exhaustTimer.hasElapsed(Feeder.FEEDER_EXHAUST_DURATION_S)) {
            transitionTo(m_preExhaustState);
        }
    }

    private void handleShootWhileIntaking() {
        // Inactive period starts: switch to alliance-wall pass automatically.
        if (HubStateMonitor.getHubState() == HubState.INACTIVE) {
            transitionTo(RobotState.PASSING_TO_ALLIANCE);
            return;
        }

        m_intake.deploy();        // keep arm deployed
        m_intake.runRoller();     // run roller only once arm is deployed (guard in IntakeSubsystem)
        m_spindexer.run();        // continuously queue balls toward feeder

        // Gate the feeder on the flywheel reaching speed for the first time this entry.
        // Once latched, feeder runs continuously for the rest of this state — same as SHOOTING.
        if (!m_feedingStarted && m_shooter.isFlywheelAtSpeed()) {
            m_feedingStarted = true;
        }
        if (m_feedingStarted) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }

        applyJamDetection();
    }

    private void handlePassingToAlliance() {
        // Active period has resumed: return to hub tracking immediately.
        // Check at the TOP so we don't command the feeder for an extra loop.
        if (HubStateMonitor.getHubState() != HubState.INACTIVE) {
            transitionTo(RobotState.PREPPING_TO_SHOOT);
            return;
        }

        // Inactive-period pass: lob all remaining balls to the alliance zone.
        // Turret angle is NOT set here — the active shoot command computes the
        // alliance-wall direction from robot pose and DriverStation.getAlliance()
        // and calls commandTurretAngle() each loop.
        m_shooter.setFlywheelRPM(SuperstructureConstants.PASS_FLYWHEEL_RPM);
        m_shooter.setHoodAngle(SuperstructureConstants.PASS_HOOD_ANGLE_DEG);
        m_intake.stopRoller();    // stop roller; arm position left to operator (may drive under trench)
        m_feeder.feed();
        m_spindexer.run();

        applyJamDetection();
    }

    private void handleTraversingTrench() {
        // Stop all scoring mechanisms every loop so any concurrently running shoot
        // command (whose execute() runs after periodic()) cannot re-energize them.
        // applyShooterSetpoint() and commandTurretAngle() are also no-ops in this state.
        m_intake.stopRoller();    // arm position left to operator for manual low-profile control
        m_shooter.stopFlywheel();
        m_shooter.stopHood();
        m_turret.stop();
        m_feeder.stop();
        m_spindexer.stop();
    }

    // =========================================================================
    // Anti-Jam Detection (shared by SHOOTING, SHOOT_WHILE_INTAKING, PASSING_TO_ALLIANCE)
    // =========================================================================

    /**
     * Monitors feeder and spindexer stator current and transitions to
     * {@link RobotState#EXHAUSTING} when a sustained jam is detected.
     *
     * <p>A jam is declared when either motor exceeds its threshold for longer than
     * {@link Feeder#FEEDER_JAM_DURATION_S} continuously.  The timer resets if
     * current drops below threshold before the duration elapses.
     *
     * <p>Must be called at the end of every handler that runs the feeder or spindexer.
     */
    private void applyJamDetection() {
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

    // =========================================================================
    // Telemetry
    // =========================================================================

    private void publishTelemetry() {
        SmartDashboard.putString( "Superstructure/State",            m_state.name());
        SmartDashboard.putBoolean("Superstructure/ReadyToShoot",     isReadyToShoot());
        SmartDashboard.putNumber( "Superstructure/BallCount",        m_ballCount);
        SmartDashboard.putNumber( "Superstructure/FlywheelRPM",      m_shooter.getFlywheelRPM());
        SmartDashboard.putNumber( "Superstructure/TargetRPM",        m_shooter.getTargetFlywheelRPM());
        SmartDashboard.putNumber( "Superstructure/HoodAngleDeg",     m_shooter.getHoodAngleDeg());
        SmartDashboard.putNumber( "Superstructure/TargetHoodDeg",    m_shooter.getTargetHoodAngleDeg());
        SmartDashboard.putNumber( "Superstructure/TurretAngleDeg",   m_turret.getAngleDeg());
        SmartDashboard.putNumber( "Superstructure/TargetTurretDeg",  m_turret.getTargetAngleDeg());
        SmartDashboard.putBoolean("Superstructure/FlywheelAtSpeed",  m_shooter.isFlywheelAtSpeed());
        SmartDashboard.putBoolean("Superstructure/HoodAtAngle",      m_shooter.isHoodAtAngle());
        SmartDashboard.putBoolean("Superstructure/TurretAligned",    m_turret.isAligned());
        SmartDashboard.putBoolean("Superstructure/FeedingStarted",   m_feedingStarted);
        SmartDashboard.putBoolean("Superstructure/JamTimerRunning",  m_jamTimerRunning);
    }

    // =========================================================================
    // State Transition
    // =========================================================================

    /**
     * Executes immediate entry actions for the given state then updates the
     * current-state field.  Entry actions stop time-sensitive actuators right
     * away so the robot does not wait a full loop for the new handler to run —
     * critical when two command end()/initialize() calls occur in the same
     * scheduler tick without an intervening periodic() call.
     *
     * @param newState The state to transition into.
     */
    private void transitionTo(RobotState newState) {
        switch (newState) {
            case STOWED -> {
                // Immediately halt all time-sensitive actuators.
                // handleStowed() re-asserts these every loop, but stopping here
                // ensures mechanisms halt within the same scheduler tick.
                m_feeder.stop();
                m_spindexer.stop();
                m_shooter.stopFlywheel();
                m_intake.stopRoller();
            }
            case INTAKING -> {
                // Immediately stop the feeder and spindexer.  Without this, a
                // direct SHOOTING → INTAKING transition (both commands' end/init
                // running in the same scheduler tick) would leave the feeder
                // running for one full loop during handleIntaking().
                m_feeder.stop();
                m_spindexer.stop();
            }
            case PREPPING_TO_SHOOT -> {
                // Stop the feeder immediately; spindexer will start in handlePrepping().
                m_feeder.stop();
            }
            case SHOOTING -> {
                m_jamTimerRunning = false;
                m_jamTimer.stop();
                m_jamTimer.reset();
            }
            case EXHAUSTING -> {
                // Start timing the exhaust cycle.  handleExhausting() begins reversing
                // motors on the next periodic() call.
                m_exhaustTimer.restart();
            }
            case TRAVERSING_TRENCH -> {
                // Immediately stop all scoring mechanisms on TRENCH entry.
                // Intake arm is NOT auto-stowed — operator controls deploy/stow.
                m_feeder.stop();
                m_spindexer.stop();
                m_shooter.stopFlywheel();
                m_shooter.stopHood();
                m_turret.stop();
                m_intake.stopRoller();
            }
            case SHOOT_WHILE_INTAKING -> {
                m_feedingStarted  = false; // re-arm the spin-up gate for this entry
                m_jamTimerRunning = false;
                m_jamTimer.stop();
                m_jamTimer.reset();
            }
            case PASSING_TO_ALLIANCE -> {
                m_jamTimerRunning = false;
                m_jamTimer.stop();
                m_jamTimer.reset();
            }
        }

        m_state = newState;
    }
}
