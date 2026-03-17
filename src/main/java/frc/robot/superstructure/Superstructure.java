package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Feeder;
import frc.robot.Constants.Shooter;
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

    // EMA state for chassis velocity — same constants as ShootCommand.
    // y_n = α·x_n + (1−α)·y_{n−1}
    private double  m_filtVx    = 0.0;
    private double  m_filtVy    = 0.0;
    private double  m_filtOmega = 0.0;
    private boolean m_sotmSeeded = false;

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

        // Full tracking rate = ω + vLateral/d
        // vLateral: robot-frame lateral velocity of turret pivot, perpendicular to hub.
        // Accounts for both translation and the rotation×offset cross term.
        var spd = m_drivetrain.getState().Speeds;
        double omega = spd.omegaRadiansPerSecond;
        double vxT = spd.vxMetersPerSecond - omega * Turret.TURRET_OFFSET_Y_M;
        double vyT = spd.vyMetersPerSecond + omega * Turret.TURRET_OFFSET_X_M;
        double hubRad    = Math.toRadians(trimmedAngleDeg); // robot-relative hub direction proxy
        double vLateral  = -vxT * Math.sin(hubRad) + vyT * Math.cos(hubRad);
        double distM     = m_photonVision.getHubDistanceMeters()
                              .or(() -> m_vision.getFusedHubDistanceMeters())
                              .orElse(4.0);
        double trackingRate = omega + vLateral / distM;
        m_turret.setAngle(trimmedAngleDeg, trackingRate);
    }

    /**
     * Same as {@link #commandTurretAngle(double)} but accepts a pre-computed
     * tracking rate from the caller.  Use this from {@link frc.robot.commands.ShootCommand}
     * which already has an exact {@code vLateral} (computed with the true hub direction,
     * not the target-angle proxy) and the current distance.
     *
     * @param angleDeg          Target turret angle (robot-relative degrees).
     * @param vLateral          Turret-pivot lateral velocity in robot frame (m/s).
     *                          Already includes rotation×offset cross terms.
     * @param distanceM         Current turret-to-hub distance (metres, clamped by caller).
     */
    public void commandTurretAngle(double angleDeg, double vLateral, double distanceM) {
        if (m_state == RobotState.TRAVERSING_TRENCH) return;
        double trimmedAngleDeg = angleDeg + Turret.TURRET_AIM_TRIM_DEG;

        double rawEncoderDeg = -trimmedAngleDeg;
        if (m_state == RobotState.SHOOTING
                && (rawEncoderDeg > Turret.TURRET_FORWARD_LIMIT_DEG
                    || rawEncoderDeg < Turret.TURRET_REVERSE_LIMIT_DEG)
                && !m_turret.isAligned()) {
            transitionTo(RobotState.WRAPAROUND);
        }

        double omega = m_drivetrain.getState().Speeds.omegaRadiansPerSecond;
        double trackingRate = omega + vLateral / Math.max(0.5, distanceM);
        m_turret.setAngle(trimmedAngleDeg, trackingRate);
    }

    /**
     * Same as {@link #commandTurretAngle(double, double, double)} but accepts the
     * numerically differentiated fire-direction rate {@code alphaFireDotRadPerSec}
     * directly as the tracking-rate feedforward.
     *
     * <p>Use this from {@link frc.robot.commands.ShootCommand} which computes
     * {@code alphaFireDot = Δα / dt} each loop.  This captures lead-angle drift
     * (e.g. during robot acceleration) that the analytical {@code ω + vLateral/d}
     * formula misses.
     *
     * @param alphaFireDotRadPerSec  Time derivative of the fire direction (rad/s).
     */
    public void commandTurretAngle(double angleDeg, double vLateral, double distanceM,
                                    double alphaFireDotRadPerSec) {
        if (m_state == RobotState.TRAVERSING_TRENCH) return;
        double trimmedAngleDeg = angleDeg + Turret.TURRET_AIM_TRIM_DEG;

        double rawEncoderDeg = -trimmedAngleDeg;
        if (m_state == RobotState.SHOOTING
                && (rawEncoderDeg > Turret.TURRET_FORWARD_LIMIT_DEG
                    || rawEncoderDeg < Turret.TURRET_REVERSE_LIMIT_DEG)
                && !m_turret.isAligned()) {
            transitionTo(RobotState.WRAPAROUND);
        }

        m_turret.setAngle(trimmedAngleDeg, alphaFireDotRadPerSec);
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
     * Returns whether the flywheel is within its moving tracking tolerance.
     * Used by {@link frc.robot.commands.ShootCommand} as the sole readiness gate
     * when the robot is moving — turret and hood tolerances would prevent firing
     * during continuous SOTM since their setpoints shift every loop.
     */
    public boolean isFlywheelTracking() {
        return m_shooter.isFlywheelTracking();
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
        // Keep turret pre-aimed at hub while stowed so there is no slew delay
        // when the operator presses shoot.  ShootCommand is not active here.
        commandTurretToHub();
    }

    private void handlePrepping() {
        // Intake deploy and roller are fully operator-controlled — not touched here.
        // Only prepare aiming mechanisms (turret, hood, flywheel).  Feeder and
        // spindexer stay stopped until the system transitions to SHOOTING.
        m_feeder.stop();
        m_spindexer.stop();
        // Full SOTM hub tracking (same math as ShootCommand).
        // ShootCommand.execute() runs AFTER periodic() and writes the same target,
        // so there is no conflict — both compute identically.
        commandTurretToHub();
    }

    /**
     * Commands the turret toward the HUB using the full SOTM pipeline identical to
     * {@link frc.robot.commands.ShootCommand}: latency compensation → iterative future
     * hub solve → exact 2D vector decomposition.
     *
     * <p>Running this every periodic() loop means the turret is pre-aimed with the
     * correct lead angle during PREPPING_TO_SHOOT, so there is no slew delay when
     * SHOOTING starts.
     */
    private void commandTurretToHub() {
        // ── Vision: hub angle + distance ─────────────────────────────────────
        double hubAngleDeg;
        double rawDistanceM;
        var odoAngle = m_vision.getHubRobotRelativeAngleDeg();
        if (odoAngle.isPresent()) {
            hubAngleDeg  = odoAngle.get();
            rawDistanceM = m_vision.getFusedHubDistanceMeters().orElse(4.0);
        } else {
            var pvAngle = m_photonVision.getHubAngleDeg();
            if (pvAngle.isEmpty()) return;
            hubAngleDeg  = pvAngle.get();
            rawDistanceM = m_photonVision.getHubDistanceMeters().orElse(4.0);
        }

        // ── EMA velocity ──────────────────────────────────────────────────────
        ChassisSpeeds rawSpd = m_drivetrain.getState().Speeds;
        if (!m_sotmSeeded) {
            m_filtVx     = rawSpd.vxMetersPerSecond;
            m_filtVy     = rawSpd.vyMetersPerSecond;
            m_filtOmega  = rawSpd.omegaRadiansPerSecond;
            m_sotmSeeded = true;
        } else {
            m_filtVx    += Shooter.SOTM_VEL_ALPHA   * (rawSpd.vxMetersPerSecond     - m_filtVx);
            m_filtVy    += Shooter.SOTM_VEL_ALPHA   * (rawSpd.vyMetersPerSecond     - m_filtVy);
            m_filtOmega += Shooter.SOTM_OMEGA_ALPHA * (rawSpd.omegaRadiansPerSecond - m_filtOmega);
        }
        double vxT = m_filtVx - m_filtOmega * Turret.TURRET_OFFSET_Y_M;
        double vyT = m_filtVy + m_filtOmega * Turret.TURRET_OFFSET_X_M;

        double alphaNowRad   = Math.toRadians(hubAngleDeg);
        boolean isStationary = Math.hypot(m_filtVx, m_filtVy) < Shooter.SOTM_SPEED_DEADBAND_MPS
                            && Math.abs(m_filtOmega) < 0.15;

        double distanceM;
        double alphaFireRad;

        // Turret pivot offset in robot frame — vision measures from robot center,
        // but ShooterKinematics needs distance from the turret pivot.
        Translation2d turretOffset = new Translation2d(Turret.TURRET_OFFSET_X_M, Turret.TURRET_OFFSET_Y_M);

        if (isStationary) {
            Translation2d hubVec = new Translation2d(rawDistanceM, new Rotation2d(alphaNowRad))
                    .minus(turretOffset);
            distanceM    = hubVec.getNorm();
            alphaFireRad = hubVec.getAngle().getRadians();
        } else {
            Translation2d pivotVel = new Translation2d(vxT, vyT);

            // ── Step 1: Latency compensation + turret offset correction ──────
            Translation2d hub = new Translation2d(rawDistanceM, new Rotation2d(alphaNowRad))
                    .rotateBy(new Rotation2d(-m_filtOmega * Shooter.SOTM_LATENCY_S))
                    .minus(pivotVel.times(Shooter.SOTM_LATENCY_S))
                    .minus(turretOffset); // robot-center → turret-pivot frame

            // ── Step 2: Iterative future hub solve (2 iterations) ─────────────
            distanceM = hub.getNorm();
            ShooterSetpoint spNow = ShooterKinematics.calculate(distanceM);
            Translation2d futHub  = hub;
            for (int i = 0; i < 2; i++) {
                double clampedD = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                                  Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, distanceM));
                double tof = ShooterKinematics.getFlightTimeSeconds(clampedD, spNow);
                futHub    = hub.minus(pivotVel.times(tof));
                distanceM = futHub.getNorm();
                spNow     = ShooterKinematics.calculate(distanceM);
            }
            distanceM = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                        Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, distanceM));

            // ── Step 3: Virtual Goal — aim at future hub, no vector subtraction ─
            // Flywheel output + robot velocity = correct ground-frame ball velocity.
            // Subtracting pivotVel again would double-compensate for the same motion.
            alphaFireRad = futHub.getAngle().getRadians();
        }

        // ── Cable-limit clamp ─────────────────────────────────────────────────
        double finalTarget = Math.toDegrees(alphaFireRad);
        double finalEncChk = -(finalTarget + Turret.TURRET_AIM_TRIM_DEG);
        if (finalEncChk > Turret.TURRET_FORWARD_LIMIT_DEG) {
            finalTarget  = -Turret.TURRET_FORWARD_LIMIT_DEG - Turret.TURRET_AIM_TRIM_DEG;
            alphaFireRad = Math.toRadians(finalTarget);
        } else if (finalEncChk < Turret.TURRET_REVERSE_LIMIT_DEG) {
            finalTarget  = -Turret.TURRET_REVERSE_LIMIT_DEG - Turret.TURRET_AIM_TRIM_DEG;
            alphaFireRad = Math.toRadians(finalTarget);
        }

        // ── Tracking feedforward ──────────────────────────────────────────────
        double vLateral = -vxT * Math.sin(alphaFireRad) + vyT * Math.cos(alphaFireRad);
        commandTurretAngle(finalTarget, vLateral, distanceM);
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

        // Save ShootCommand's SOTM turret target from the previous loop.
        // WPILib runs periodic() before execute(), so m_turret.getTargetAngleDeg() here
        // is the angle ShootCommand.execute() wrote last loop — the correct SOTM lead target.
        // ShootCommand owns turret in SHOOTING state; commandTurretToHub() is NOT called
        // here to avoid overwriting ShootCommand's target before it re-asserts this loop.
        double shootCmdTurretTarget = m_turret.getTargetAngleDeg();

        // Turret gate: check BOTH current error and predicted future error, take the
        // minimum.  Using only the prediction causes false failures during continuous
        // fire: when the turret is faithfully tracking a continuously-moving target
        // (velocity ≈ target rate), the prediction overshoots past the CURRENT target
        // even though the turret is correctly on it.
        //   e.g. turret at 15°, target 15°, vel = 30 °/s tracking robot rotation:
        //        predAngle = 15 + 30×0.08 = 17.4° → |17.4−15| > tolerance → feeder stops.
        // Taking min(currentError, predError) handles both cases:
        //   • First shot (turret slewing to lead): predError detects early arrival.
        //   • Continuous fire (turret on target with tracking velocity): currentError ≈ 0 → passes.
        double turretCurrentAngle = m_turret.getAngleDeg();
        double turretPredAngle    = turretCurrentAngle
                + m_turret.getVelocityDegPerSec() * Shooter.FEEDER_TRANSIT_SECONDS;
        double turretCurrentError = Math.abs(turretCurrentAngle - shootCmdTurretTarget);
        double turretPredError    = Math.abs(turretPredAngle    - shootCmdTurretTarget);
        boolean turretPredReady   = Math.min(turretCurrentError, turretPredError)
                < Turret.TURRET_MOVING_TOLERANCE_DEG;

        SmartDashboard.putNumber("Shoot/TurretCurrentErrorDeg", turretCurrentError);
        SmartDashboard.putNumber("Shoot/TurretPredErrorDeg",    turretPredError);
        SmartDashboard.putBoolean("Shoot/TurretPredReady",      turretPredReady);

        ChassisSpeeds shootSpeeds = m_drivetrain.getState().Speeds;
        boolean isMoving = Math.hypot(shootSpeeds.vxMetersPerSecond,
                                      shootSpeeds.vyMetersPerSecond) > 0.1
                        || Math.abs(shootSpeeds.omegaRadiansPerSecond) > 0.3;
        // Moving: flywheel + turret on ShootCommand's SOTM target.
        // CRITICAL: use turretPredReady (based on shootCmdTurretTarget saved above) NOT
        // m_turret.isTracking().  commandTurretToHub() just wrote a fresh basic hub target
        // to the turret; isTracking() compares against THAT target — not the SOTM lead
        // angle ShootCommand commanded.  When lead > TURRET_MOVING_TOLERANCE_DEG (3°) the
        // turret can be perfectly on the SOTM target while isTracking() returns false,
        // cutting the feeder on every loop and halving effective fire rate.
        // turretPredReady = min(currentError, predError) < tolerance, referenced to the
        // previous loop's SOTM target — correctly handles both continuous fire (currentError ≈ 0)
        // and the initial slew to the lead angle (predError detects early arrival).
        // Hood accuracy is inherent in the SOTM setpoint — no tracking check needed for moving.
        // Stationary: all three mechanisms must be tracking for the precise first shot.
        boolean canFeed = isMoving
                ? (m_shooter.isFlywheelTracking() && turretPredReady)
                : (m_shooter.isFlywheelTracking() && m_shooter.isHoodTracking()
                   && turretPredReady);

        if (canFeed) {
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

        // Return to PREPPING once the turret is within tracking tolerance.
        // Using isTracking() (wide tolerance) instead of isAligned() (tight tolerance)
        // is critical when the robot is moving: the target shifts every loop, so the
        // turret never fully "aligns" within the tight static tolerance — isAligned()
        // would keep the system stuck in WRAPAROUND for the entire match.
        // isTracking() exits as soon as the turret is close enough; the full readiness
        // re-check (isTrackingSetpoints) in ShootCommand gates the PREPPING→SHOOTING
        // transition before the first shot resumes.
        if (m_turret.isTracking()) {
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
        // Shooter setpoints (RPM + hood) and turret angle are provided each loop
        // by PassCommand via applyShooterSetpoint() and commandTurretAngle() —
        // same pattern as handleShooting().  PassCommand applies the same
        // Hybrid-TOF SOTM algorithm (with latency compensation) so passes are
        // accurate whether the robot is stationary or moving.
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
