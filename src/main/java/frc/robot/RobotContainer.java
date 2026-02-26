package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.HomeTurretCommand;
import frc.robot.commands.HubAlignCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OrientedDriveCommand;
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
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;

/**
 * Wires all subsystems, commands, and controller bindings together.
 *
 * <h2>Controller Layout (port 0 — Xbox)</h2>
 * <pre>
 *  Left Stick          — Field-centric translation (default)
 *  Left Stick Click    — Oriented drive (robot-centric, 30% speed)
 *  Right Stick X       — Rotation
 *  A                   — X-brake (lock wheels)
 *  B                   — Point wheels toward left stick direction
 *  X (no Back)         — Navigate to HUB approach pose (PathFinder)
 *  Left Bumper         — Seed field-centric heading
 *  Left Trigger        — Intake (deploy + run spindexer; drive speed capped to roller surface speed)
 *  Left Trigger + Right Bumper — Shoot while intaking (continuous, moving-while-shooting; drive speed capped)
 *  Right Bumper        — Shoot (prep + fire when ready)
 *  Right Trigger       — Intake under TRENCH (roller only, low height)
 *  POV Up / Down       — Robot-centric forward / reverse (slow)
 *  Back + A            — Home turret (run once per power-up)
 *  Back + B            — Pass through TRENCH (stow all, robot-centric drive)
 *  Back + Y            — SysId dynamic forward
 *  Back + X            — SysId dynamic reverse
 *  Start + Y           — SysId quasistatic forward
 *  Start + X           — SysId quasistatic reverse
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

    private final CommandXboxController joystick = new CommandXboxController(0);

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
     * Beam-break sensor mounted between the feeder exit and the flywheel contact zone.
     * The sensor reads LOW ({@code false}) while a ball is breaking the beam and
     * HIGH ({@code true}) when clear.  The rising edge (ball fully exits the beam)
     * is used as the shot-confirmed signal to decrement the ball count.
     *
     * <p>Update {@link Shooter#SHOOTER_BEAM_BREAK_DIO_PORT} to match physical wiring.
     */
    private final DigitalInput m_shooterBeamBreak =
            new DigitalInput(Shooter.SHOOTER_BEAM_BREAK_DIO_PORT);

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
                        .withVelocityX(-joystick.getLeftY()  * MaxSpeed * speedFraction)
                        .withVelocityY(-joystick.getLeftX()  * MaxSpeed * speedFraction)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
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

        // --- Drive utility bindings ------------------------------------------
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.povUp().whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(
            drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // --- Oriented drive mode (left stick click) --------------------------
        // Switches to robot-centric drive at 30% speed for tight manoeuvres.
        joystick.leftStick().whileTrue(
            new OrientedDriveCommand(drivetrain,
                joystick::getLeftY, joystick::getLeftX, joystick::getRightX,
                MaxSpeed, MaxAngularRate)
        );

        // --- Hub alignment (X button, Back NOT held) -------------------------
        // Pathfinds the robot to a pose in front of the alliance HUB.
        // The Back modifier is excluded so Back+X can still trigger SysId reverse.
        joystick.x().and(joystick.back().negate()).whileTrue(
            HubAlignCommand.create(drivetrain, m_superstructure));

        // --- Scoring bindings ------------------------------------------------

        // Left Trigger + Right Bumper → shoot while intaking (interrupts individual intake/shoot).
        // Registered first so that when both buttons are held, this combined command takes over
        // (WPILib cancels individual bindings that require the same subsystem).
        joystick.leftTrigger().and(joystick.rightBumper()).whileTrue(
            new ShootWhileIntakingCommand(m_superstructure, m_vision, drivetrain)
        );

        // Left trigger → deploy intake and run spindexer.
        joystick.leftTrigger().whileTrue(new IntakeCommand(m_superstructure));

        // Right bumper → prep shooter and fire when ready.
        joystick.rightBumper().whileTrue(
            new ShootCommand(m_superstructure, m_vision, drivetrain)
        );

        // Right trigger → intake under TRENCH (roller only at reduced height).
        joystick.rightTrigger().whileTrue(new IntakeUnderTrenchCommand(m_superstructure));

        // --- TRENCH bindings -------------------------------------------------

        // Back + B → manually command TRENCH transit (stow all mechanisms).
        joystick.back().and(joystick.b()).whileTrue(
            new PassThroughTrenchCommand(m_superstructure)
        );

        // --- Turret homing ---------------------------------------------------
        // Back + A → home turret (run once after power-up).
        // Pass a real BooleanSupplier for the limit switch when hardware is installed;
        // null enables stall-detect homing as the fallback.
        joystick.back().and(joystick.a()).onTrue(
            new HomeTurretCommand(m_turret, null /* limit switch supplier */)
        );

        // --- Drivetrain SysId  (Back/Start + Y/X) ----------------------------
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // --- Mechanism SysId  (Back/Start + POV Left/Right) ------------------
        // Select the mechanism to characterize via the "Mechanism SysId Routine"
        // chooser on SmartDashboard before enabling in Test mode.
        //   Back  + POV Left  → dynamic forward
        //   Back  + POV Right → dynamic reverse
        //   Start + POV Left  → quasistatic forward
        //   Start + POV Right → quasistatic reverse
        joystick.back().and(joystick.povLeft()).whileTrue(
            Commands.defer(() -> m_mechanismSysId.getSelected().dynamic(Direction.kForward),
                java.util.Set.of()));
        joystick.back().and(joystick.povRight()).whileTrue(
            Commands.defer(() -> m_mechanismSysId.getSelected().dynamic(Direction.kReverse),
                java.util.Set.of()));
        joystick.start().and(joystick.povLeft()).whileTrue(
            Commands.defer(() -> m_mechanismSysId.getSelected().quasistatic(Direction.kForward),
                java.util.Set.of()));
        joystick.start().and(joystick.povRight()).whileTrue(
            Commands.defer(() -> m_mechanismSysId.getSelected().quasistatic(Direction.kReverse),
                java.util.Set.of()));

        // --- Shot counter (beam-break) ---------------------------------------
        // Rising edge: sensor goes HIGH (beam clear) after being blocked by a ball.
        // Each rising edge = one ball confirmed shot → decrement ball count.
        // ignoringDisable so the counter still works if somehow triggered while DS
        // is not enabled (e.g. during a connection blip between periods).
        new edu.wpi.first.wpilibj2.command.button.Trigger(m_shooterBeamBreak::get)
                .onTrue(Commands.runOnce(m_superstructure::decrementBallCount)
                                .ignoringDisable(true));

        // --- Telemetry -------------------------------------------------------
        drivetrain.registerTelemetry(logger::telemeterize);
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
