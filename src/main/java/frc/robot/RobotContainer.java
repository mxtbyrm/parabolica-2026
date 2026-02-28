package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.HomeTurretCommand;
import frc.robot.commands.SystemHealthCheckCommand;
import frc.robot.commands.HubAlignCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OrientedDriveCommand;
import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootWhileIntakingCommand;
import frc.robot.commands.trench.IntakeUnderTrenchCommand;
import frc.robot.commands.trench.PassThroughTrenchCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FaultMonitor;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TrenchTraversalManager;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;

/**
 * Wires all subsystems, commands, and controller bindings together.
 *
 * <h2>Driver Controller (port 0 — Xbox)</h2>
 * <pre>
 *  Left Stick          — Field-centric translation (default)
 *  Left Stick Click    — Oriented drive (robot-centric, 30% speed)
 *  Right Stick X       — Rotation
 *  A                   — X-brake (lock wheels)
 *  B                   — Point wheels toward left stick direction
 *  X                   — Navigate to HUB approach pose (PathFinder)
 *  Left Bumper         — Seed field-centric heading
 *  Start               — Reset odometry to hub front (visionless pose seed)
 *  POV Up / Down       — Robot-centric forward / reverse (slow)
 *  Back + B            — Pass through TRENCH (stow all, robot-centric drive)
 * </pre>
 *
 * <h2>Operator Controller (port 1 — Xbox) — Teleop / Auto</h2>
 * <pre>
 *  Left Trigger        — Intake (deploy + run spindexer; drive speed capped to roller surface speed)
 *  Left Trigger + Right Bumper — Shoot while intaking (continuous, moving-while-shooting; drive speed capped)
 *  Right Bumper        — Shoot (prep + fire when ready)
 *  Right Trigger       — Intake under TRENCH (roller only, low height)
 *  Y (no Back/Start)   — Stow intake arm
 *  Back + A            — Home turret (run once per power-up)
 *  Back + Y            — SysId dynamic forward
 *  Back + X            — SysId dynamic reverse
 *  Start + Y           — SysId quasistatic forward
 *  Start + X           — SysId quasistatic reverse
 * </pre>
 *
 * <h2>Operator Controller (port 1 — Xbox) — Test Mode (direct subsystem control)</h2>
 * <pre>
 *  Left Stick X        — Turret: direct percent output (±30% speed)
 *  POV Up / Down       — Hood: step angle +2° / -2°
 *  Left Trigger        — Flywheel: spin at 2 000 RPM
 *  Right Trigger       — Exhaust: reverse feeder + spindexer
 *  A                   — Feeder: forward
 *  B                   — Spindexer: forward
 *  X                   — Intake roller: run
 *  Left Bumper         — Intake arm: deploy
 *  Right Bumper        — Intake arm: stow
 *  Back + A            — Home turret
 * </pre>
 */
public class RobotContainer {

    // =========================================================================
    // Drive constants
    // =========================================================================

    private final double MaxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // =========================================================================
    // Drive requests
    // =========================================================================

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake      = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt    point      = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric     forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // =========================================================================
    // Controllers
    // =========================================================================

    private final CommandXboxController driver   = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // =========================================================================
    // Drivetrain  (must be declared before VisionSubsystem which depends on it)
    // =========================================================================

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // =========================================================================
    // Scoring Subsystems
    // =========================================================================

    private final ShooterSubsystem   m_shooter   = new ShooterSubsystem();
    private final TurretSubsystem    m_turret    = new TurretSubsystem();
    private final FeederSubsystem    m_feeder    = new FeederSubsystem();
    private final SpindexerSubsystem m_spindexer = new SpindexerSubsystem();
    private final IntakeSubsystem    m_intake    = new IntakeSubsystem();

    // =========================================================================
    // Sensors
    // =========================================================================

    /**
     * CANrange proximity sensor mounted between the feeder exit and the flywheel
     * contact zone.  Reports distance in metres; a reading below
     * {@link Shooter#SHOOTER_CANRANGE_THRESHOLD_M} indicates a ball is present.
     * A ball transitioning from present → absent (rising edge of the "ball gone"
     * condition) is used as the shot-confirmed signal to decrement the ball count.
     *
     * <p>Update {@link Shooter#SHOOTER_CANRANGE_CAN_ID} to match the physical CAN ID.
     */
    private final CANrange m_shooterCanRange =
            new CANrange(Shooter.SHOOTER_CANRANGE_CAN_ID, "CANivore");

    // =========================================================================
    // Vision  (constructed after drivetrain)
    // =========================================================================

    /**
     * VisionSubsystem feeds pose estimates into {@link #drivetrain} each loop and
     * provides HUB targeting data directly to {@link #m_superstructure} for
     * always-on turret tracking, and to ShootCommand for moving-while-shooting
     * compensation.
     */
    private final VisionSubsystem m_vision = new VisionSubsystem(drivetrain);

    /**
     * Four corner-mounted PhotonVision cameras that feed global pose measurements
     * into the drivetrain pose estimator.  Runs alongside {@link #m_vision}
     * (Limelight); the two systems are independent.  Falls back to odometry-only
     * automatically when all cameras are offline or see no tags.
     */
    private final PhotonVisionSubsystem m_photonVision = new PhotonVisionSubsystem(drivetrain);

    // =========================================================================
    // Superstructure
    // =========================================================================

    /**
     * Central state machine coordinating all scoring subsystems.
     *
     * <p><b>Shot counting</b> — the Superstructure uses continuous-fire mode and
     * does not count shots automatically.  Install a beam-break sensor at the
     * shooter exit and call {@link Superstructure#decrementBallCount()} from its
     * trigger to maintain an accurate ball count.
     */
    private final Superstructure m_superstructure = new Superstructure(
            m_shooter, m_turret, m_feeder, m_spindexer, m_intake, m_vision);

    // =========================================================================
    // Fault Monitor  (instantiated after all motor subsystems)
    // =========================================================================

    /**
     * Polls all motor subsystems and the CANivore bus each loop for critical faults.
     * Faults are surfaced as WPILib Alerts on the DriverStation.
     * {@link FaultMonitor#hasAnyCriticalFault()} can be used to gate safety-critical actions.
     */
    private final FaultMonitor m_faultMonitor = new FaultMonitor(
            m_shooter, m_turret, m_feeder, m_spindexer, m_intake);

    // =========================================================================
    // TRENCH Traversal Manager
    // =========================================================================

    /**
     * Automatically stows mechanisms and requests TRAVERSING_TRENCH when the robot
     * approaches or enters a TRENCH structure.  Works in parallel with driver commands.
     */
    private final TrenchTraversalManager m_trenchManager =
            new TrenchTraversalManager(drivetrain, m_superstructure);

    // =========================================================================
    // Telemetry + Autonomous
    // =========================================================================

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    /**
     * Selects which mechanism SysId routine to run when the mechanism SysId
     * buttons are pressed (Back/Start + POV Left/Right).
     *
     * <p>Options are populated in the constructor and displayed on SmartDashboard
     * under "Mechanism SysId Routine".  Change the selection before enabling in
     * Test mode to characterize the desired subsystem.
     */
    private final SendableChooser<SysIdRoutine> m_mechanismSysId = new SendableChooser<>();

    // =========================================================================
    // Constructor
    // =========================================================================

    public RobotContainer() {
        // Register named commands for PathPlanner autos BEFORE building the chooser.
        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Populate mechanism SysId chooser — select via SmartDashboard before
        // enabling in Test mode.  Back/Start + POV Left/Right triggers the routine.
        m_mechanismSysId.setDefaultOption("Flywheel",        m_shooter.getSysIdFlywheelRoutine());
        m_mechanismSysId.addOption(       "Hood",            m_shooter.getSysIdHoodRoutine());
        m_mechanismSysId.addOption(       "Turret",          m_turret.getSysIdRoutine());
        m_mechanismSysId.addOption(       "Intake Deploy",   m_intake.getSysIdDeployRoutine());
        m_mechanismSysId.addOption(       "Feeder",          m_feeder.getSysIdRoutine());
        m_mechanismSysId.addOption(       "Spindexer",       m_spindexer.getSysIdRoutine());
        SmartDashboard.putData("Mechanism SysId Routine", m_mechanismSysId);

        // SmartDashboard button — click while enabled to run the full health check.
        SmartDashboard.putData("System Health Check",
                new SystemHealthCheckCommand(
                        m_shooter, m_turret, m_feeder, m_spindexer, m_intake, m_superstructure));

        // Publish the pre-load ball count as a SmartDashboard number so the driver
        // can adjust it from the DS before each match without redeploying code.
        SmartDashboard.putNumber("Preload Ball Count", SuperstructureConstants.PRELOAD_BALL_COUNT);

        configureBindings();

        // Warm up PathPlanner's JIT to avoid first-path latency spikes.
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    // =========================================================================
    // Match Preparation
    // =========================================================================

    /**
     * Presets the ball count to the number of balls physically loaded at match start.
     * Call this from {@link frc.robot.Robot#autonomousInit()} and
     * {@link frc.robot.Robot#teleopInit()} so the counter is accurate from the
     * first shot.
     *
     * <p>The count is read from the SmartDashboard entry "Preload Ball Count"
     * (editable on the driver station); it defaults to
     * {@link SuperstructureConstants#PRELOAD_BALL_COUNT} if the entry is absent.
     */
    public void prepareForMatch() {
        // Always reset to STOWED at the start of every period so the Superstructure
        // never carries active-firing state across a disable boundary.  Without this,
        // a match ending in SHOOTING would resume with the feeder running at teleop start.
        m_superstructure.requestState(Superstructure.RobotState.STOWED);

        int preload = (int) SmartDashboard.getNumber(
                "Preload Ball Count", SuperstructureConstants.PRELOAD_BALL_COUNT);
        m_superstructure.setBallCount(preload);
    }

    // =========================================================================
    // Named Commands (PathPlanner autos)
    // =========================================================================

    /**
     * Registers all named commands that PathPlanner autos can reference.
     * Call this before {@link AutoBuilder#buildAutoChooser} so the auto chooser
     * can validate the command names.
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("Intake",
            new IntakeCommand(m_superstructure));

        NamedCommands.registerCommand("Shoot",
            new AutoShootCommand(m_superstructure, m_vision, drivetrain));

        NamedCommands.registerCommand("PassThroughTrench",
            new PassThroughTrenchCommand(m_superstructure));

        NamedCommands.registerCommand("IntakeUnderTrench",
            new IntakeUnderTrenchCommand(m_superstructure));
    }

    // =========================================================================
    // Bindings
    // =========================================================================

    private void configureBindings() {

        // --- Default drive command -------------------------------------------
        // While the intake is deployed, cap translation speed below the roller
        // contact surface speed (adjusted for motor load).  If the robot drives
        // faster than the roller can pull, it pushes balls instead of intaking them.
        // Rotation rate is intentionally uncapped — spinning in place does not
        // affect ball-approach velocity.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                Superstructure.RobotState state = m_superstructure.getState();
                boolean intaking = state == Superstructure.RobotState.INTAKING
                                || state == Superstructure.RobotState.SHOOT_WHILE_INTAKING;
                // Clamp to [0, 1]: if roller surface speed exceeds robot max, no cap applies.
                double speedFraction = intaking
                        ? Math.min(1.0, Intake.MAX_DRIVE_SPEED_WHILE_INTAKING_MPS / MaxSpeed)
                        : 1.0;
                return drive
                        .withVelocityX(-driver.getLeftY()  * MaxSpeed * speedFraction)
                        .withVelocityY(-driver.getLeftX()  * MaxSpeed * speedFraction)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate);
            })
        );

        // --- Disabled coast mode --------------------------------------------
        // When the robot is disabled, switch to coast mode so it can be pushed
        // safely between matches.  Switch back to brake mode on enable.
        RobotModeTriggers.disabled().onTrue(
            drivetrain.runOnce(() -> drivetrain.configureNeutralMode(NeutralModeValue.Coast))
                      .ignoringDisable(true)
        );
        RobotModeTriggers.disabled().onFalse(
            drivetrain.runOnce(() -> drivetrain.configureNeutralMode(NeutralModeValue.Brake))
        );

        // Idle (neutral) while disabled so configured neutral mode is respected.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // --- Drive utility bindings (driver controller) ----------------------
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driver.povUp().whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.povDown().whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // --- Oriented drive mode (left stick click) --------------------------
        driver.leftStick().whileTrue(
            new OrientedDriveCommand(drivetrain,
                driver::getLeftY, driver::getLeftX, driver::getRightX,
                MaxSpeed, MaxAngularRate)
        );

        // --- Hub alignment ---------------------------------------------------
        driver.x().whileTrue(HubAlignCommand.create(drivetrain, m_superstructure));

        // --- Hub front pose reset (visionless) --------------------------------
        // Physically place the robot directly in front of the hub at
        // MIN_SHOOT_RANGE_M, facing the hub, then press Start to seed the
        // odometry to that known position.  When vision is enabled, this is
        // redundant (vision corrects the pose automatically) but always safe to use.
        driver.start().onTrue(Commands.runOnce(() -> {
            boolean isBlue = DriverStation.getAlliance()
                    .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
            Translation2d hub = isBlue
                    ? FieldLayout.BLUE_HUB_CENTER
                    : FieldLayout.RED_HUB_CENTER;
            double resetDist = SuperstructureConstants.MIN_SHOOT_RANGE_M;
            // Blue: robot is on the alliance-wall side of the hub, faces +X toward hub.
            // Red:  robot is on the alliance-wall side of the hub, faces -X toward hub.
            double poseX     = hub.getX() + (isBlue ? -resetDist : +resetDist);
            double facingDeg = isBlue ? 0.0 : 180.0;
            drivetrain.resetPose(new Pose2d(poseX, hub.getY(),
                    Rotation2d.fromDegrees(facingDeg)));
        }).ignoringDisable(true));

        // --- Turret default command ------------------------------------------
        // The turret's default command (runs whenever no other command requires it)
        // provides stick control in Test mode.  Using a default command — rather than
        // test.whileTrue() — ensures the stick resumes automatically after any
        // interrupting command (e.g. HomeTurretCommand) finishes without having to
        // leave and re-enter Test mode.  In Teleop/Auto the lambda is a no-op so it
        // does not fight the Superstructure, which drives the turret directly each loop.
        m_turret.setDefaultCommand(Commands.run(
            () -> {
                if (DriverStation.isTest()) {
                    m_turret.driveAtPercent(operator.getLeftX() * 0.3);
                }
                // teleop / auto: Superstructure owns the turret via direct method calls
            },
            m_turret
        ));

        // --- Scoring bindings (operator controller — teleop/auto only) -------
        // All bindings guarded with notTest so Test mode can control subsystems directly.
        Trigger notTest = RobotModeTriggers.test().negate();

        // Left Trigger + Right Bumper → shoot while intaking.
        // Registered first so when both are held this command takes priority.
        operator.leftTrigger().and(operator.rightBumper()).and(notTest).whileTrue(
            new ShootWhileIntakingCommand(m_superstructure, m_vision, drivetrain)
        );

        // Left Trigger → deploy intake and run spindexer.
        operator.leftTrigger().and(notTest).whileTrue(new IntakeCommand(m_superstructure));

        // Right Bumper → prep shooter and fire when ready.
        operator.rightBumper().and(notTest).whileTrue(
            new ShootCommand(m_superstructure, m_vision, drivetrain)
        );

        // Right Trigger → intake under TRENCH (roller only at reduced height).
        operator.rightTrigger().and(notTest).whileTrue(new IntakeUnderTrenchCommand(m_superstructure));

        // Y → explicit intake stow (Back+Y / Start+Y reserved for SysId).
        operator.y().and(operator.back().negate()).and(operator.start().negate()).and(notTest).onTrue(
            Commands.runOnce(m_intake::stow, m_intake)
        );

        // --- Turret homing (operator controller) -----------------------------
        operator.back().and(operator.a()).and(notTest).onTrue(
            new HomeTurretCommand(m_turret)
        );

        // --- SysId (operator controller — Back/Start + Y/X) ------------------
        operator.back().and(operator.y()).and(notTest).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        operator.back().and(operator.x()).and(notTest).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        operator.start().and(operator.y()).and(notTest).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        operator.start().and(operator.x()).and(notTest).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // --- Mechanism SysId (operator Back/Start + POV Left/Right) ----------
        operator.back().and(operator.povLeft()).and(notTest).whileTrue(
            Commands.defer(() -> m_mechanismSysId.getSelected().dynamic(Direction.kForward),
                java.util.Set.of()));
        operator.back().and(operator.povRight()).and(notTest).whileTrue(
            Commands.defer(() -> m_mechanismSysId.getSelected().dynamic(Direction.kReverse),
                java.util.Set.of()));
        operator.start().and(operator.povLeft()).and(notTest).whileTrue(
            Commands.defer(() -> m_mechanismSysId.getSelected().quasistatic(Direction.kForward),
                java.util.Set.of()));
        operator.start().and(operator.povRight()).and(notTest).whileTrue(
            Commands.defer(() -> m_mechanismSysId.getSelected().quasistatic(Direction.kReverse),
                java.util.Set.of()));

        // --- Test mode — direct subsystem control ----------------------------
        configureTestBindings();

        // --- Shot counter (CANrange) -----------------------------------------
        // Disabled in Constants until the sensor is physically wired and confirmed.
        // Flip Shooter.SHOOTER_CANRANGE_ENABLED to true to activate.
        // "Ball present" when distance < threshold.  A transition from present →
        // absent (onFalse of the ballPresent trigger) = ball has fully cleared the
        // sensor = shot confirmed → decrement ball count.
        if (Shooter.SHOOTER_CANRANGE_ENABLED) {
            new edu.wpi.first.wpilibj2.command.button.Trigger(
                    () -> m_shooterCanRange.getDistance().getValueAsDouble()
                            < Shooter.SHOOTER_CANRANGE_THRESHOLD_M)
                    .onFalse(Commands.runOnce(m_superstructure::decrementBallCount)
                                     .ignoringDisable(true));
        }

        // --- Telemetry -------------------------------------------------------
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // =========================================================================
    // Test Mode — direct subsystem control (bypasses Superstructure)
    // =========================================================================

    /**
     * Registers operator-controller bindings that are active ONLY in Test mode.
     * Every binding is ANDed with {@link RobotModeTriggers#test()} so they are
     * completely silent during teleop and auto.  Normal operator bindings are
     * ANDed with {@code test().negate()} so they go silent here.
     *
     * <p>Use Test mode on the Driver Station to verify each mechanism independently
     * without the Superstructure state machine interfering.
     */
    private void configureTestBindings() {
        Trigger test = RobotModeTriggers.test();

        // Turret: left stick X → handled by the turret default command set in
        // configureBindings().  No whileTrue binding needed here.

        // Hood: POV Up / Down → step angle ±2°
        operator.povUp().and(test).onTrue(Commands.runOnce(
                () -> m_shooter.setHoodAngle(m_shooter.getHoodAngleDeg() + 2.0), m_shooter));
        operator.povDown().and(test).onTrue(Commands.runOnce(
                () -> m_shooter.setHoodAngle(m_shooter.getHoodAngleDeg() - 2.0), m_shooter));

        // Flywheel: Left Trigger → 2 000 RPM; stops when released.
        operator.leftTrigger().and(test).whileTrue(
                Commands.run(() -> m_shooter.setFlywheelRPM(2000.0), m_shooter)
                        .finallyDo(() -> m_shooter.stopFlywheel()));

        // Feeder: A → forward; stops when released.
        operator.a().and(test).whileTrue(
                Commands.run(m_feeder::feed, m_feeder)
                        .finallyDo(() -> m_feeder.stop()));

        // Spindexer: B → forward; stops when released.
        operator.b().and(test).whileTrue(
                Commands.run(m_spindexer::run, m_spindexer)
                        .finallyDo(() -> m_spindexer.stop()));

        // Exhaust: Right Trigger → reverse feeder + spindexer; stops when released.
        operator.rightTrigger().and(test).whileTrue(
                Commands.run(() -> { m_feeder.reverse(); m_spindexer.reverse(); },
                             m_feeder, m_spindexer)
                        .finallyDo(() -> { m_feeder.stop(); m_spindexer.stop(); }));

        // Intake roller: X → run roller only (arm must already be deployed; guard is in runRoller()).
        operator.x().and(test).whileTrue(
                Commands.run(m_intake::runRoller, m_intake)
                        .finallyDo(() -> m_intake.stopRoller()));

        // Intake arm: Left Bumper → deploy, Right Bumper → stow.
        operator.leftBumper().and(test).onTrue(Commands.runOnce(m_intake::deploy, m_intake));
        operator.rightBumper().and(test).onTrue(Commands.runOnce(m_intake::stow, m_intake));

        // Turret home: Back + A (same as teleop).
        operator.back().and(operator.a()).and(test).onTrue(
                new HomeTurretCommand(m_turret));
    }

    // =========================================================================
    // Autonomous
    // =========================================================================

    /**
     * Returns the autonomous command selected via the SmartDashboard chooser.
     *
     * @return The selected auto command, or {@code null} if none is selected.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
