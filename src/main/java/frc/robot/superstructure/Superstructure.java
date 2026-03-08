package frc.robot.superstructure;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Feeder;
import frc.robot.Constants.Spindexer;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
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
 *   <li>{@link RobotState#STOWED} — All scoring mechanisms at rest.  Intake is
 *       fully operator-controlled and not managed by the Superstructure.</li>
 *   <li>{@link RobotState#PREPPING_TO_SHOOT} — Turret tracking HUB; flywheel and
 *       hood at calculated setpoints; feeder and spindexer stopped.  Does not
 *       fire — awaiting full readiness ({@link #isReadyToShoot()}).</li>
 *   <li>{@link RobotState#SHOOTING} — Continuous fire; spindexer runs; feeder
 *       gated on {@link #isTrackingSetpoints()}.  Anti-jam monitoring active.</li>
 *   <li>{@link RobotState#WRAPAROUND} — Turret is executing a large slew
 *       (detected proactively when {@link #commandTurretAngle} receives a target
 *       requiring travel exceeding the wraparound threshold).  Flywheel and hood
 *       maintain setpoints; feeder and spindexer stopped.  Returns to
 *       PREPPING_TO_SHOOT when turret is aligned so a full readiness check is
 *       enforced before firing resumes.</li>
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
 * <p>Once in SHOOTING, the spindexer runs continuously and the feeder is gated
 * on {@link #isTrackingSetpoints()} so balls are only fed when the flywheel,
 * hood, and turret are within moving tolerances.  The flywheel setpoint is
 * updated every loop by the active shoot command; the high-inertia steel
 * flywheel wheels maintain speed between shots and the control loop recovers
 * during ball transit.  No per-ball stop-and-wait cycle is used.
 *
 * <h2>Anti-Jam Logic</h2>
 * <p>While in SHOOTING or PASSING_TO_ALLIANCE, the
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
        /** All scoring mechanisms idle.  Intake is fully operator-controlled. */
        STOWED,

        /**
         * Turret locked on target; flywheel and hood at calculated setpoints;
         * feeder and spindexer stopped.  Will not fire — awaiting readiness.
         */
        PREPPING_TO_SHOOT,

        /**
         * Continuous active firing: spindexer and feeder run when mechanisms
         * are tracking setpoints ({@link #isTrackingSetpoints()}).
         * Flywheel setpoint updated every loop by the active shoot command.
         * Anti-jam monitoring active; jams transition to EXHAUSTING and back.
         */
        SHOOTING,

        /**
         * Turret wraparound: detected proactively when
         * {@link Superstructure#commandTurretAngle} receives a target whose
         * raw encoder angle falls outside the cable-travel limits
         * ({@link Turret#TURRET_REVERSE_LIMIT_DEG} … {@link Turret#TURRET_FORWARD_LIMIT_DEG}),
         * meaning {@code inputModulus} would map it to the far end of the range.
         * The turret slews to the new target while flywheel and hood maintain
         * their setpoints; feeder and spindexer stopped.  Automatically returns
         * to PREPPING_TO_SHOOT once the turret is aligned.
         */
        WRAPAROUND,

        /**
         * Jam recovery: feeder and spindexer reversed for a fixed duration,
         * then the machine returns to PREPPING_TO_SHOOT so a full readiness
         * re-check is enforced before firing resumes.
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
         * Inactive-period pass: turret faces alliance wall (commanded by the active
         * shoot command), hood and flywheel at fixed pass setpoints, feeder/spindexer
         * lob balls to the alliance zone.
         * Entered automatically from SHOOTING when
         * {@link HubState#INACTIVE} is detected.  Exits automatically back to
         * PREPPING_TO_SHOOT when the active period resumes.
         */
        PASSING_TO_ALLIANCE
    }

    // =========================================================================
    // Subsystem References
    // =========================================================================

    private final CommandSwerveDrivetrain m_drivetrain;
    private final ShooterSubsystem        m_shooter;
    private final TurretSubsystem         m_turret;
    private final FeederSubsystem         m_feeder;
    private final SpindexerSubsystem      m_spindexer;
    private final VisionSubsystem         m_vision;
    private final PhotonVisionSubsystem   m_photonVision;

    // =========================================================================
    // State Machine Variables
    // =========================================================================

    private RobotState m_state           = RobotState.STOWED;

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
     * <p>The intake subsystem is <em>not</em> included — intake deploy/stow and
     * roller are fully operator-controlled during teleop via independent bindings
     * in {@link frc.robot.RobotContainer}.
     *
     * @param shooter    The flywheel and hood subsystem.
     * @param turret     The turret rotation subsystem.
     * @param feeder     The ball feeder subsystem.
     * @param spindexer  The spindexer disk subsystem.
     * @param vision     The vision subsystem (HUB targeting).
     * @param photonVision The four corner-camera subsystem (raw-pose hub angle).
     */
    public Superstructure(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem        shooter,
            TurretSubsystem         turret,
            FeederSubsystem         feeder,
            SpindexerSubsystem      spindexer,
            VisionSubsystem         vision,
            PhotonVisionSubsystem   photonVision) {
        m_drivetrain   = drivetrain;
        m_shooter      = shooter;
        m_turret       = turret;
        m_feeder       = feeder;
        m_spindexer    = spindexer;
        m_vision       = vision;
        m_photonVision = photonVision;
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
            case PREPPING_TO_SHOOT     -> handlePrepping();
            case SHOOTING              -> handleShooting();
            case WRAPAROUND            -> handleWraparound();
            case EXHAUSTING            -> handleExhausting();
            case TRAVERSING_TRENCH     -> handleTraversingTrench();
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
     * <p><b>Proactive wraparound detection:</b> While in SHOOTING, this method
     * converts the requested robot-relative angle to encoder space and checks
     * whether it falls outside the cable-travel limits
     * ({@link Turret#TURRET_REVERSE_LIMIT_DEG} … {@link Turret#TURRET_FORWARD_LIMIT_DEG}).
     * If it does, {@code inputModulus} inside {@link TurretSubsystem#setAngle} would
     * wrap the target to the far end of the range — a large physical slew.  The state
     * machine transitions to {@link RobotState#WRAPAROUND} <em>before</em> the motor
     * is commanded so the feeder and spindexer stop within the same scheduler tick.
     * The turret is still commanded so it begins slewing immediately; WRAPAROUND →
     * PREPPING_TO_SHOOT ensures full readiness is re-verified before firing resumes.
     *
     * <p><b>No-op while in {@link RobotState#TRAVERSING_TRENCH}.</b>
     *
     * @param angleDeg Target turret angle in degrees (0 = forward, positive = CCW).
     */
    public void commandTurretAngle(double angleDeg) {
        if (m_state == RobotState.TRAVERSING_TRENCH) return;

        // Apply static aim trim to compensate for systematic bias (encoder zero
        // offset, PhotonVision heading error, turret pivot misalignment, etc.).
        double trimmedAngleDeg = angleDeg + Turret.TURRET_AIM_TRIM_DEG;

        // Proactive wraparound: convert to encoder space (encoder = -robot-relative)
        // and check whether the raw value lies outside the cable-travel range.
        // If it does, TurretSubsystem.setAngle() will apply inputModulus to wrap it
        // to the opposite end of the range, meaning the turret must travel the long
        // way around the cable — stop feeding immediately before that happens.
        double rawEncoderDeg = -trimmedAngleDeg;
        if (m_state == RobotState.SHOOTING
                && (rawEncoderDeg > Turret.TURRET_FORWARD_LIMIT_DEG
                    || rawEncoderDeg < Turret.TURRET_REVERSE_LIMIT_DEG)
                && !m_turret.isAligned()) {
            // Only trigger WRAPAROUND if the turret is NOT already at the target.
            // Without this guard, a target whose raw encoder angle falls outside
            // the cable-travel range (e.g. hub at -90° robot-relative →
            // rawEncoder = +90° > FORWARD_LIMIT) would trigger WRAPAROUND on
            // every SHOOTING loop even after the turret has already completed the
            // long-path slew and is sitting exactly on target — permanently
            // preventing the feeder from firing from the side of the hub.
            transitionTo(RobotState.WRAPAROUND);
        }

        m_turret.setAngle(trimmedAngleDeg);
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
     * Returns whether flywheel, hood, and turret have fully settled at their
     * setpoints using tight static tolerances.
     *
     * <p>Gates the PREPPING_TO_SHOOT → SHOOTING transition in
     * {@link frc.robot.commands.ShootCommand}.  All three mechanisms must
     * converge before firing begins when stationary.  While the robot is moving,
     * uses wider tracking tolerances since setpoints shift every loop.
     *
     * @return {@code true} if flywheel, hood, and turret are within tight static tolerance.
     */
    public boolean isReadyToShoot() {
        return m_shooter.isFlywheelAtSpeed()
            && m_shooter.isHoodAtAngle()
            && m_turret.isAligned();
    }

    /**
     * Returns whether flywheel, hood, and turret are within the wider <em>moving</em>
     * tolerances, used to gate the feeder during SHOOTING so balls are only fed
     * when mechanisms are close to their setpoints.
     *
     * <p>Uses {@link frc.robot.subsystems.ShooterSubsystem#isFlywheelTracking()},
     * {@link frc.robot.subsystems.ShooterSubsystem#isHoodTracking()}, and
     * {@link frc.robot.subsystems.TurretSubsystem#isTracking()} so the robot can
     * sustain shooting while moving without requiring mechanisms to fully settle
     * at tight static tolerances.  The turret uses a wider moving tolerance
     * ({@link frc.robot.Constants.Turret#TURRET_MOVING_TOLERANCE_DEG}) because
     * its setpoint shifts every loop during SOTM; the 1° static tolerance would
     * cut the feeder on nearly every loop.
     *
     * <p>The PREPPING_TO_SHOOT → SHOOTING transition uses the tighter
     * {@link #isReadyToShoot()} gate instead.
     *
     * @return {@code true} if mechanisms are tracking their setpoints closely enough
     *         to continue feeding.
     */
    public boolean isTrackingSetpoints() {
        return m_shooter.isFlywheelTracking()
            && m_shooter.isHoodTracking()
            && m_turret.isTracking();
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

    /** @return The flywheel's current measured RPM (pass-through for telemetry). */
    public double getFlywheelRPM() { return m_shooter.getFlywheelRPM(); }

    // =========================================================================
    // State Handlers
    // =========================================================================

    private void handleStowed() {
        // Intake is fully operator-controlled — not touched here.
        m_shooter.stopFlywheel();
        m_shooter.stopHood();
        m_feeder.stop();
        m_spindexer.stop();
        // Turret tracks the hub so it is already pointed when the operator
        // requests PREPPING_TO_SHOOT.  Full fallback chain: PV → LL → odometry.
        commandTurretToHub();
    }

    private void handlePrepping() {
        // Intake deploy and roller are fully operator-controlled — not touched here.
        // Only prepare aiming mechanisms (turret, hood, flywheel).  Feeder and
        // spindexer stay stopped until the system transitions to SHOOTING.
        m_feeder.stop();
        m_spindexer.stop();

        // Basic hub tracking.  ShootCommand.execute() runs AFTER periodic() each loop
        // and overrides this with full moving-while-shooting compensation (radial d_eff
        // + lateral lead angle + turret pivot offset) when it is active.  This handler
        // provides continuous turret pointing when no shoot command is running.
        commandTurretToHub();
    }

    /**
     * Points the turret toward the hub, including a lateral lead angle to
     * compensate for the robot's current chassis velocity.  Called every loop
     * from {@link #handleStowed()} and {@link #handlePrepping()} so the turret
     * is already pre-aimed when the operator presses shoot.
     *
     * <p>Angle priority: PhotonVision → Odometry.
     * Distance priority: PhotonVision → Odometry.
     */
    private void commandTurretToHub() {
        double hubAngleDeg;
        double distanceM;

        // Odometry (fused pose estimator) is PRIMARY — as documented in
        // VisionSubsystem.getHubRobotRelativeAngleDeg().  The fused pose already
        // incorporates all available vision corrections (Limelight + PhotonVision)
        // via the Kalman filter, computed from the true turret pivot position with
        // no EMA lag.  PhotonVision's EMA-filtered PnP angle can lag behind the
        // true direction when the robot repositions quickly, and is subject to
        // systematic PnP errors from oblique viewing angles (e.g. right side of hub).
        // PhotonVision is used only when alliance is unknown (pre-match / sim).
        var odoAngle = m_vision.getHubRobotRelativeAngleDeg();
        if (odoAngle.isPresent()) {
            hubAngleDeg = odoAngle.get();
            distanceM   = m_vision.getFusedHubDistanceMeters().orElse(4.0);
        } else {
            var pvAngle = m_photonVision.getHubAngleDeg();
            if (pvAngle.isEmpty()) return;
            hubAngleDeg = pvAngle.get();
            distanceM   = m_photonVision.getHubDistanceMeters().orElse(4.0);
        }

        // Skip lead-angle compensation when the turret is mid-slew (error > 20°).
        // During a long CCW slew (e.g. a 270° cable-wrap path), getAngleDeg()
        // returns mid-slew values like +135° or +180° (turret physically pointing
        // backward).  At those angles vLateral in computeLeadAngleDeg() is computed
        // from the wrong direction, producing a completely wrong lead angle every
        // loop.  commandTurretAngle() then chases an oscillating setpoint instead
        // of completing the slew — causing the turret to hunt wildly.
        // Once error ≤ 20° the turret is close enough that getAngleDeg() is a
        // valid proxy for the shot direction and lead-angle compensation resumes.
        double leadAngleDeg = m_turret.getErrorDeg() > 20.0
                ? 0.0
                : computeLeadAngleDeg(distanceM);
        commandTurretAngle(hubAngleDeg + leadAngleDeg);
    }

    /**
     * Computes the lateral lead angle (degrees) to compensate for robot motion
     * during ball flight.  Uses raw chassis speeds — no filtering.
     */
    private double computeLeadAngleDeg(double distanceM) {
        ChassisSpeeds spd = m_drivetrain.getState().Speeds;
        double turretRad  = Math.toRadians(m_turret.getAngleDeg());

        // Velocity at the turret pivot = robot center velocity + ω × offset.
        double vxT = spd.vxMetersPerSecond   - spd.omegaRadiansPerSecond * Turret.TURRET_OFFSET_Y_M;
        double vyT = spd.vyMetersPerSecond   + spd.omegaRadiansPerSecond * Turret.TURRET_OFFSET_X_M;

        double vRadial  =  vxT * Math.cos(turretRad) + vyT * Math.sin(turretRad);
        double vLateral = -vxT * Math.sin(turretRad) + vyT * Math.cos(turretRad);

        ShooterSetpoint sp = ShooterKinematics.calculate(distanceM);
        double tFlight = ShooterKinematics.getFlightTimeSeconds(distanceM, sp, vRadial);

        if (distanceM <= 0.0 || tFlight <= 0.0) return 0.0;
        return Math.toDegrees(Math.atan2(-vLateral * tFlight, distanceM));
    }

    private void handleShooting() {
        // Inactive period starts: switch to alliance-wall pass automatically.
        if (HubStateMonitor.getHubState() == HubState.INACTIVE) {
            transitionTo(RobotState.PASSING_TO_ALLIANCE);
            return;
        }

        // NOTE: Wraparound detection is handled proactively inside
        // commandTurretAngle() — when the requested encoder angle falls outside
        // the cable-travel limits the state transitions to WRAPAROUND *before*
        // the motor is commanded.  No reactive error check is needed here.

        // Intake deploy and roller are fully operator-controlled — not touched here.

        // --- Energy gate (flywheel + hood only) --------------------------------
        // Gate the feeder on flywheel speed and hood angle only — NOT turret.
        // Rationale: turret aim accuracy is the responsibility of ShootCommand's
        // setpoint loop; checking turret error here cuts the feeder every time
        // the turret lags even slightly behind a moving setpoint, producing
        // intermittent fire.  The PREPPING→SHOOTING transition (via
        // isTrackingSetpoints()) already required the turret to be aimed before
        // the first shot.  Large turret slews are caught by proactive WRAPAROUND
        // detection in commandTurretAngle() and stop the feeder that way.
        // The spindexer runs ONLY when the feeder is actively feeding — if the
        // feeder stops, the spindexer stops too so balls don't jam at the
        // feeder throat.
        if (m_shooter.isFlywheelTracking() && m_shooter.isHoodTracking()) {
            m_feeder.feed();
            m_spindexer.run();
        } else {
            m_feeder.stop();
            m_spindexer.stop();
        }

        applyJamDetection();
    }

    private void handleWraparound() {
        // Flywheel and hood keep running at their setpoints so they are ready
        // the instant the turret arrives.  ShootCommand.execute() continues to
        // call applyShooterSetpoint() and commandTurretAngle() every loop.
        m_feeder.stop();
        m_spindexer.stop();

        // Return to PREPPING once the turret is within alignment tolerance.
        // ShootCommand will then re-evaluate full readiness (flywheel, hood,
        // turret, distance, hub state) before transitioning back to SHOOTING.
        if (m_turret.isAligned()) {
            transitionTo(RobotState.PREPPING_TO_SHOOT);
        }
    }

    private void handleExhausting() {
        m_feeder.reverse();
        m_spindexer.reverse();
        if (m_exhaustTimer.hasElapsed(Feeder.FEEDER_EXHAUST_DURATION_S)) {
            // Always return to PREPPING_TO_SHOOT after a jam clear — even if the
            // jam occurred during SHOOTING.  The flywheel will have slowed during
            // the exhaust reversal, so a full readiness re-check (isReadyToShoot)
            // must pass before firing resumes.  For PASSING_TO_ALLIANCE the
            // setpoints are fixed and recover quickly, but the PREPPING gate
            // ensures the flywheel is actually back at speed before feeding again.
            transitionTo(RobotState.PREPPING_TO_SHOOT);
        }
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
        //
        // Read live-tuneable setpoints from SmartDashboard so the operator can
        // adjust RPM and hood angle on the fly without redeploying code.
        double passRPM      = SmartDashboard.getNumber("Pass/FlywheelRPM",
                SuperstructureConstants.PASS_FLYWHEEL_RPM);
        double passHoodDeg  = SmartDashboard.getNumber("Pass/HoodAngleDeg",
                SuperstructureConstants.PASS_HOOD_ANGLE_DEG);
        m_shooter.setFlywheelRPM(passRPM);
        m_shooter.setHoodAngle(passHoodDeg);

        // Gate the feeder on flywheel speed — do NOT feed balls into a stopped or
        // slow flywheel.  Without this gate the first balls after entering
        // PASSING_TO_ALLIANCE dribble out or jam because the flywheel hasn't
        // spun up yet.  Same pattern as handleShooting().
        if (m_shooter.isFlywheelTracking() && m_shooter.isHoodTracking()) {
            m_feeder.feed();
            m_spindexer.run();
        } else {
            m_feeder.stop();
            m_spindexer.stop();
        }

        applyJamDetection();
    }

    private void handleTraversingTrench() {
        // Stop all scoring mechanisms every loop so any concurrently running shoot
        // command (whose execute() runs after periodic()) cannot re-energize them.
        // applyShooterSetpoint() and commandTurretAngle() are also no-ops in this state.
        // Intake deploy and roller are fully operator-controlled — not touched here.
        m_shooter.stopFlywheel();
        m_shooter.stopHood();
        m_turret.stop();
        m_feeder.stop();
        m_spindexer.stop();
    }

    // =========================================================================
    // Anti-Jam Detection (shared by SHOOTING, PASSING_TO_ALLIANCE)
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
        SmartDashboard.putBoolean("Superstructure/TrackingSetpoints", isTrackingSetpoints());
        SmartDashboard.putBoolean("Superstructure/TurretAligned",    m_turret.isAligned());
        SmartDashboard.putBoolean("Superstructure/JamTimerRunning",  m_jamTimerRunning);
        SmartDashboard.putNumber( "Superstructure/TurretErrorDeg",   m_turret.getErrorDeg());
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
                // Immediately halt scoring actuators within this scheduler tick.
                // handleStowed() re-asserts these every loop, but stopping here
                // ensures mechanisms halt without a one-loop lag.
                m_feeder.stop();
                m_spindexer.stop();
                m_shooter.stopFlywheel();
                m_shooter.stopHood();
                // Turret is NOT stopped — handleStowed() immediately re-commands it
                // toward the hub via vision so it is already aimed on next PREPPING entry.
                // Intake is fully operator-controlled — not touched on transitions.
            }
            case PREPPING_TO_SHOOT -> {
                // Stop feeder and spindexer immediately so no balls are fed during
                // the one-loop gap before handlePrepping() runs.
                m_feeder.stop();
                m_spindexer.stop();
            }
            case SHOOTING -> {
                m_jamTimerRunning = false;
                m_jamTimer.stop();
                m_jamTimer.reset();
            }
            case WRAPAROUND -> {
                // Stop feeder and spindexer immediately; flywheel/hood keep running.
                m_feeder.stop();
                m_spindexer.stop();
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
                // Intake is fully operator-controlled — not touched on transitions.
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
