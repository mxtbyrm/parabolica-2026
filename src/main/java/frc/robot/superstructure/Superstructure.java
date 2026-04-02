package frc.robot.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Field;
import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.Feeder;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Spindexer;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
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
 *   <li>{@link RobotState#PREPPING_TO_SHOOT} — ShootCommand owns turret tracking;
 *       feeder and spindexer stopped.  Does not fire — awaiting full readiness
 *       ({@link #isReadyToShoot()}).</li>
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
         * Intake rack is between stowed and agitate — turret held at 0° to prevent
         * collision with the linear gear rack.  All scoring mechanisms stopped.
         * Entered automatically from any state (except {@link #TRAVERSING_TRENCH})
         * when {@link Superstructure#isIntakeSafeForTurret()} returns {@code false}.
         * Exits automatically back to {@link #STOWED} once the rack clears agitate.
         * {@link #applyShooterSetpoint} and {@link #commandTurretAngle} are no-ops here.
         */
        INTAKE_UNSAFE,

        /**
         * Turret locked on pass target; flywheel and hood at pass setpoints;
         * feeder and spindexer stopped.  PassCommand owns the transition to
         * PASSING_TO_ALLIANCE once {@link #isReadyToPass()} returns true.
         */
        PREPPING_TO_PASS,

        /**
         * Inactive-period pass: turret faces alliance wall (commanded by the active
         * shoot command), hood and flywheel at fixed pass setpoints, feeder/spindexer
         * lob balls to the alliance zone.
         * Entered manually by operator command when the hub is inactive.
         * Exits back to PREPPING_TO_SHOOT when the operator resumes shooting.
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
    private final IntakeSubsystem         m_intake;

    // =========================================================================
    // State Machine Variables
    // =========================================================================

    private RobotState m_state           = RobotState.STOWED;

    // Vision dropout persistence — propagate angle + distance when vision is lost.
    private double m_lastAlphaNowRad = 0.0;
    private double m_lastDistanceM   = 4.0;

    // Real dt tracking for commandTurretToHub().
    private double m_lastTimestamp = 0.0;

    // State to return to after WRAPAROUND completes — PREPPING_TO_SHOOT or PREPPING_TO_PASS.
    private RobotState m_preWraparoundState = RobotState.PREPPING_TO_SHOOT;

    // Feed latch — once canFeed goes true in SHOOTING, hold feeder/spindexer on
    // continuously until SHOOTING exits.  Prevents flickering from SOTM target jitter.
    private boolean m_feedLatch = false;

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
     * <p>The intake position is monitored to gate turret hub-tracking — when the
     * rack is between stowed and agitate the turret is held at 0° to prevent it
     * from striking the linear gear rack.
     *
     * @param shooter    The flywheel and hood subsystem.
     * @param turret     The turret rotation subsystem.
     * @param feeder     The ball feeder subsystem.
     * @param spindexer  The spindexer disk subsystem.
     * @param vision     The vision subsystem (HUB targeting).
     * @param photonVision The four corner-camera subsystem (raw-pose hub angle).
     * @param intake     The intake deploy subsystem (position read for turret safety gate).
     */
    public Superstructure(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem        shooter,
            TurretSubsystem         turret,
            FeederSubsystem         feeder,
            SpindexerSubsystem      spindexer,
            VisionSubsystem         vision,
            PhotonVisionSubsystem   photonVision,
            IntakeSubsystem         intake) {
        m_drivetrain   = drivetrain;
        m_shooter      = shooter;
        m_turret       = turret;
        m_feeder       = feeder;
        m_spindexer    = spindexer;
        m_vision       = vision;
        m_photonVision = photonVision;
        m_intake       = intake;
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

        // Intake safety pre-check: if the rack is in the danger zone enter
        // INTAKE_UNSAFE before the state handler runs so no handler ever
        // commands the turret toward the hub while the rack can be hit.
        if (!isIntakeSafeForTurret()
                && m_state != RobotState.TRAVERSING_TRENCH
                && m_state != RobotState.INTAKE_UNSAFE) {
            transitionTo(RobotState.INTAKE_UNSAFE);
        }

        switch (m_state) {
            case STOWED                -> handleStowed();
            case PREPPING_TO_SHOOT     -> handlePrepping();
            case SHOOTING              -> handleShooting();
            case WRAPAROUND            -> handleWraparound();
            case EXHAUSTING            -> handleExhausting();
            case TRAVERSING_TRENCH     -> handleTraversingTrench();
            case INTAKE_UNSAFE         -> handleIntakeUnsafe();
            case PREPPING_TO_PASS      -> handlePreppingToPass();
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
        if (m_state == RobotState.TRAVERSING_TRENCH
                || m_state == RobotState.INTAKE_UNSAFE) return;
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
        if (m_state == RobotState.TRAVERSING_TRENCH
                || m_state == RobotState.INTAKE_UNSAFE) return;
        double trimmedAngleDeg = angleDeg + Turret.TURRET_AIM_TRIM_DEG;
        if ((m_state == RobotState.SHOOTING
                    || m_state == RobotState.PASSING_TO_ALLIANCE
                    || m_state == RobotState.PREPPING_TO_SHOOT
                    || m_state == RobotState.PREPPING_TO_PASS)
                && m_turret.getRequiredTravelDeg(trimmedAngleDeg)
                   > Turret.TURRET_WRAPAROUND_TRAVEL_THRESHOLD_DEG) {
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
     * Returns whether flywheel, hood, and turret have fully settled at their
     * pass setpoints using tight static tolerances.
     *
     * <p>Gates the PREPPING_TO_PASS → PASSING_TO_ALLIANCE transition in
     * {@link frc.robot.commands.PassCommand}.
     *
     * @return {@code true} if all three mechanisms are within tight static tolerance.
     */
    public boolean isReadyToPass() {
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
        // Pre-aim turret while stowed so there is no slew delay when the operator acts.
        // In the neutral zone (farther from alliance wall than the hub) pre-aim at the
        // pass target so a quick pass needs no extra slew time.  In the alliance zone
        // (between alliance wall and hub) pre-aim at the hub for a fast shot.
        // The intake safety gate inside commandTurretAngle() holds the turret at 0°
        // automatically when the rack is between stowed and agitate.
        if (isInNeutralZone()) {
            commandTurretToPassTarget();
        } else {
            commandTurretToHub();
        }
    }

    private void handlePrepping() {
        // Intake deploy and roller are fully operator-controlled — not touched here.
        // Only prepare aiming mechanisms (turret, hood, flywheel).  Feeder and
        // spindexer stay stopped until the system transitions to SHOOTING.
        m_feeder.stop();
        m_spindexer.stop();
        // Turret is NOT commanded here — ShootCommand.execute() owns the turret
        // exclusively while in PREPPING_TO_SHOOT and SHOOTING.  Writing the same
        // target from periodic() would be a redundant write every loop; omitting it
        // ensures only one source ever commands the turret motor.
    }

    private void handlePreppingToPass() {
        // Mirror of handlePrepping() for the pass pipeline.
        // Feeder and spindexer stay stopped until PASSING_TO_ALLIANCE.
        m_feeder.stop();
        m_spindexer.stop();
        // Turret is NOT commanded here — PassCommand.execute() owns it exclusively
        // while in PREPPING_TO_PASS and PASSING_TO_ALLIANCE.
    }

    /**
     * Returns {@code true} when the intake rack position is at or beyond the agitate
     * position (toward deployed), meaning it is safe for the turret to track the hub
     * and for shooting / passing to proceed.
     *
     * <p>When the rack is between stowed ({@link Intake#DEPLOY_STOWED_ROT}) and agitate
     * ({@link Intake#DEPLOY_AGITATE_ROT}) the linear gear rack is close enough to the
     * turret rotation path that an unconstrained slew could cause a collision.
     * The turret is commanded to 0° (forward) and state transitions to SHOOTING or
     * PASSING_TO_ALLIANCE are blocked until the rack clears the danger zone.
     *
     * @return {@code true} if hub tracking and firing are permitted.
     */
    public boolean isIntakeSafeForTurret() {
        return m_intake.getDeployPositionRot() >= Intake.DEPLOY_AGITATE_ROT;
    }


    /**
     * Returns {@code true} when the robot is in the neutral zone — farther from its
     * own alliance wall than the hub is.  In this zone a pass is more likely than a
     * shot, so the turret pre-aims at the pass target instead of the hub.
     */
    private boolean isInNeutralZone() {
        double robotX = m_drivetrain.getState().Pose.getTranslation().getX();
        boolean isRed = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        double distFromAllianceWall = isRed
                ? FieldLayout.FIELD_LENGTH_M - robotX
                : robotX;
        return distFromAllianceWall > Field.HUB_DIST_FROM_ALLIANCE_WALL_M;
    }

    /**
     * Commands the turret toward the appropriate pass target (same side-selection
     * logic as {@link frc.robot.commands.PassCommand}) using a single-lookup stationary
     * aim.  No SOTM compensation — the robot is stowed and typically slow-moving;
     * precise lead is not needed for pre-aiming.
     */
    private void commandTurretToPassTarget() {
        var robotPose = m_drivetrain.getState().Pose;
        boolean isRed = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        double robotY = robotPose.getTranslation().getY();
        boolean isRightSide = robotY < FieldLayout.FIELD_WIDTH_M / 2.0;

        Translation2d target = isRed
                ? (isRightSide ? FieldLayout.RED_PASS_TARGET_RIGHT  : FieldLayout.RED_PASS_TARGET_LEFT)
                : (isRightSide ? FieldLayout.BLUE_PASS_TARGET_RIGHT : FieldLayout.BLUE_PASS_TARGET_LEFT);

        Translation2d pivotPos = robotPose.getTranslation().plus(
                new Translation2d(Turret.TURRET_OFFSET_X_M, Turret.TURRET_OFFSET_Y_M)
                        .rotateBy(robotPose.getRotation()));

        Translation2d delta      = target.minus(pivotPos);
        Translation2d deltaRobot = delta.rotateBy(robotPose.getRotation().unaryMinus());
        double angleDeg = Math.toDegrees(Math.atan2(deltaRobot.getY(), deltaRobot.getX()));
        commandTurretAngle(angleDeg);
    }

    /**
     * Commands the turret toward the HUB using the same instantaneous SOTM pipeline
     * as {@link frc.robot.commands.ShootCommand}: raw chassis velocity → EVS lead
     * angle → exact 2D vector decomposition.  No latency prediction — drivetrain
     * pose is already fused from 4 PhotonVision cameras + odometry.
     *
     * <p>Running this every periodic() loop means the turret is pre-aimed with the
     * correct lead angle during PREPPING_TO_SHOOT, so there is no slew delay when
     * SHOOTING starts.
     */
    private void commandTurretToHub() {
        // ── Real dt ───────────────────────────────────────────────────────────
        double now = Timer.getFPGATimestamp();
        double dt  = MathUtil.clamp(now - m_lastTimestamp, 0.005, 0.040);
        m_lastTimestamp = now;

        // ── Raw chassis velocity (no filter — drivetrain pose is already fused) ──
        ChassisSpeeds rawSpd = m_drivetrain.getState().Speeds;
        double vx    = rawSpd.vxMetersPerSecond;
        double vy    = rawSpd.vyMetersPerSecond;
        double omega = rawSpd.omegaRadiansPerSecond;
        double vxT   = vx - omega * Turret.TURRET_OFFSET_Y_M;
        double vyT   = vy + omega * Turret.TURRET_OFFSET_X_M;

        // ── Vision: hub angle + distance with dropout propagation ─────────────
        double[] hubAngleDeg = {Double.NaN};
        m_vision.getHubRobotRelativeAngleDeg().ifPresent(a -> hubAngleDeg[0] = a);
        if (Double.isNaN(hubAngleDeg[0])) {
            m_photonVision.getHubAngleDeg().ifPresent(a -> hubAngleDeg[0] = a);
        }

        if (!Double.isNaN(hubAngleDeg[0])) {
            m_lastAlphaNowRad = Math.toRadians(hubAngleDeg[0]);
            double freshDist = m_vision.getFusedHubDistanceMeters()
                    .or(() -> m_photonVision.getHubDistanceMeters())
                    .orElse(m_lastDistanceM);
            if (freshDist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                    && freshDist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                m_lastDistanceM = freshDist;
            }
        } else {
            // Vision dropout: propagate angle and distance using odometry.
            m_lastAlphaNowRad = MathUtil.angleModulus(m_lastAlphaNowRad - omega * dt);
            double dRadial = vxT * Math.cos(m_lastAlphaNowRad)
                           + vyT * Math.sin(m_lastAlphaNowRad);
            m_lastDistanceM = Math.max(
                    m_lastDistanceM - dRadial * dt,
                    SuperstructureConstants.MIN_SHOOT_RANGE_M);
        }
        double alphaNowRad  = m_lastAlphaNowRad;
        double rawDistanceM = m_lastDistanceM;

        double chassisSpeedMps = Math.hypot(vx, vy);
        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS
                            && Math.abs(omega) < 0.15;

        // pivotVel: pivot velocity including omega×offset — for EVS subtraction.
        Translation2d pivotVel = new Translation2d(vxT, vyT);

        double distanceM = rawDistanceM;
        double alphaFireRad;

        if (isStationary) {
            alphaFireRad = alphaNowRad;
        } else {
            // ── EVS — keep vy0 fixed, subtract pivot velocity ─────────────────
            distanceM = MathUtil.clamp(distanceM,
                    SuperstructureConstants.MIN_SHOOT_RANGE_M,
                    SuperstructureConstants.MAX_SHOOT_RANGE_M);
            ShooterSetpoint reqShot = ShooterKinematics.calculate(distanceM);
            double v0       = ShooterKinematics.rpmToLaunchSpeed(reqShot.flywheelRPM());
            double exitRad  = Math.toRadians(90.0 - reqShot.hoodAngleDeg());
            double vHoriz   = v0 * Math.cos(exitRad);
            double vRelX    = vHoriz * Math.cos(alphaNowRad) - pivotVel.getX();
            double vRelY    = vHoriz * Math.sin(alphaNowRad) - pivotVel.getY();
            alphaFireRad    = Math.atan2(vRelY, vRelX);
        }

        commandTurretAngle(Math.toDegrees(alphaFireRad));
    }

    private void handleShooting() {
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
        // turretPredReady is referenced to the previous loop's SOTM target (from
        // shootCmdTurretTarget above).  commandTurretToHub() is only called from
        // handleStowed() — NOT here — so shootCmdTurretTarget is always the SOTM
        // lead angle ShootCommand wrote last loop.
        // min(currentError, predError) < tolerance handles both cases:
        //   • Faithful tracking: currentError ≈ 0 → always passes.
        //   • Initial slew to lead angle: predError detects early arrival.
        // Hood accuracy is inherent in the SOTM setpoint — no hood check for moving.
        // Stationary: require all three (flywheel + hood + turret) for the first shot.
        boolean canFeed = isMoving
                ? (m_shooter.isFlywheelTracking() && turretPredReady)
                : (m_shooter.isFlywheelTracking() && m_shooter.isHoodTracking()
                   && turretPredReady);

        // Latch: once conditions are met, keep feeding continuously until SHOOTING exits.
        // Prevents feeder/spindexer from cutting out due to SOTM target jitter (±1-2° per
        // loop causes turretPredReady / isFlywheelTracking to flicker at ~50 Hz).
        if (canFeed) m_feedLatch = true;

        if (m_feedLatch) {
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
            transitionTo(m_preWraparoundState);
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
        //
        // Latch: once flywheel is at speed, keep feeder/spindexer on continuously.
        // Prevents flickering from SOTM setpoint jitter (same fix as handleShooting).
        if (m_shooter.isFlywheelTracking()) m_feedLatch = true;

        if (m_feedLatch) {
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

    private void handleIntakeUnsafe() {
        // Rack is between stowed and agitate — hold turret at 0° every loop.
        // applyShooterSetpoint() and commandTurretAngle() are no-ops in this state
        // so no concurrently running shoot/pass command can override the turret.
        m_shooter.stopFlywheel();
        m_shooter.stopHood();
        m_feeder.stop();
        m_spindexer.stop();
        m_turret.setAngle(Turret.TURRET_AIM_TRIM_DEG);

        // Exit as soon as the rack clears the agitate position.
        if (isIntakeSafeForTurret()) {
            transitionTo(RobotState.STOWED);
        }
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
                m_feedLatch = false;
            }
            case PREPPING_TO_PASS -> {
                m_feeder.stop();
                m_spindexer.stop();
            }
            case WRAPAROUND -> {
                // Save return state so handleWraparound() goes back to the right
                // prep state (PREPPING_TO_SHOOT for shoot, PREPPING_TO_PASS for pass).
                m_preWraparoundState = (m_state == RobotState.PASSING_TO_ALLIANCE
                        || m_state == RobotState.PREPPING_TO_PASS)
                        ? RobotState.PREPPING_TO_PASS
                        : RobotState.PREPPING_TO_SHOOT;
                // Stop feeder and spindexer immediately; flywheel/hood keep running.
                m_feedLatch = false;
                m_feeder.stop();
                m_spindexer.stop();
            }
            case EXHAUSTING -> {
                // Start timing the exhaust cycle.  handleExhausting() begins reversing
                // motors on the next periodic() call.
                m_exhaustTimer.restart();
            }
            case INTAKE_UNSAFE -> {
                // Immediately stop all scoring mechanisms.
                // handleIntakeUnsafe() commands turret to 0° on the next periodic() call.
                m_feeder.stop();
                m_spindexer.stop();
                m_shooter.stopFlywheel();
                m_shooter.stopHood();
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
                m_feedLatch = false;
            }
        }

        m_state = newState;
    }
}
