package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.Intake;

/**
 * Manages the ground intake mechanism, which consists of:
 *
 * <ul>
 *   <li><b>Deploy pivot</b> — master (left, {@link Intake#DEPLOY_LEFT_CAN_ID}) and
 *       strict follower (right, {@link Intake#DEPLOY_RIGHT_CAN_ID}) Kraken X60 motors
 *       using MotionMagic position control to extend/retract the intake arm.</li>
 *   <li><b>Roller</b> — one Kraken X60 ({@link Intake#ROLLER_CAN_ID}) running
 *       open-loop duty-cycle to ingest or expel FUEL.</li>
 * </ul>
 *
 * <h2>Physical Layout</h2>
 * <p>The two deploy motors are mirror-mounted on opposite sides of the robot frame.
 * The left motor is the master; the right follows with
 * {@code opposeDirection = true} ({@link Intake#DEPLOY_FOLLOWER_OPPOSES_MASTER}) so
 * both produce the same physical arm motion despite their mirrored orientations.
 * Phoenix 6 ignores {@code MotorOutputConfigs.Inverted} in Follower mode; direction
 * is controlled solely by the {@code opposeDirection} flag.
 *
 * <p>The roller motor is bolted to the <em>right</em> side of the intake flap, making
 * the right deploy motor support the combined weight of the flap and the roller assembly.
 * The right motor therefore has a higher current limit than the left
 * ({@link Intake#DEPLOY_RIGHT_STATOR_LIMIT_A} vs {@link Intake#DEPLOY_LEFT_STATOR_LIMIT_A})
 * to prevent stall-limiting during stow when it must lift against that extra mass.
 *
 * <h2>Flap Behavior</h2>
 * <p>The intake flap is gravity-actuated: it opens automatically once the arm is
 * deployed (gravity pulls it down), and closes under its own weight as the arm stows.
 * No additional motor drives the flap.
 */
public class IntakeSubsystem extends SubsystemBase {

    // ── Deploy direction tracking (for asymmetric Motion Magic profiles) ──────
    // true  = stowing (lifting against gravity) — faster cruise, extra FF
    // false = deploying (falling with gravity)  — slow cruise
    private boolean m_isStowing = false;

    // Hardware
    private final TalonFX m_deployLeft  = new TalonFX(Intake.DEPLOY_LEFT_CAN_ID);
    private final TalonFX m_deployRight = new TalonFX(Intake.DEPLOY_RIGHT_CAN_ID);
    private final TalonFX m_roller      = new TalonFX(Intake.ROLLER_CAN_ID);

    // Control requests
    private final MotionMagicVoltage m_deployPositionReq =
            new MotionMagicVoltage(0).withSlot(0);
    // opposeDirection = true: Phoenix 6 ignores MotorOutputConfigs.Inverted in Follower mode.
    // The flag is the ONLY way to counter-rotate the mirror-mounted right motor.
    private final Follower m_followerReq =
            new Follower(Intake.DEPLOY_LEFT_CAN_ID, Intake.DEPLOY_FOLLOWER_OPPOSES_MASTER);
    private final DutyCycleOut m_rollerDutyCycleReq = new DutyCycleOut(0);
    private final NeutralOut   m_neutralReq          = new NeutralOut();
    private final VoltageOut   m_deployVoltageReq    = new VoltageOut(0);

    // SysId routine for intake deploy arm position characterization.
    // Only the master (left) motor is driven; the follower tracks it automatically.
    private final SysIdRoutine m_sysIdRoutineDeploy = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // default ramp rate (1 V/s)
            Volts.of(4), // conservative step voltage — limited-range position mechanism
            null,        // default timeout (10 s)
            state -> SignalLogger.writeString("SysIdIntakeDeploy_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> m_deployLeft.setControl(m_deployVoltageReq.withOutput(volts.in(Volts))),
            null,
            this
        )
    );

    /**
     * Constructs the IntakeSubsystem and applies all motor configurations.
     * Must be instantiated once, inside {@link frc.robot.RobotContainer}.
     */
    public IntakeSubsystem() {
        configureDeployMotors();
        configureRollerMotor();
        // Seed the deploy encoder assuming the arm is at its stowed position on boot.
        // Only the master encoder matters for control; the follower tracks output directly.
        m_deployLeft.setPosition(
                Units.degreesToRotations(Intake.DEPLOY_STOWED_DEG) * Intake.DEPLOY_GEAR_RATIO);
    }

    // -------------------------------------------------------------------------
    // Public Command Methods
    // -------------------------------------------------------------------------

    /**
     * Moves the intake arm to the deployed (ground-contact) position.
     * Does <em>not</em> start the roller — call {@link #runRoller()} separately.
     * The arm travels to {@link Intake#DEPLOY_DEPLOYED_DEG} via MotionMagic.
     */
    public void deploy() {
        setDeployAngle(Intake.DEPLOY_DEPLOYED_DEG);
    }

    /**
     * Moves the intake arm to the stowed (retracted) position and stops the roller.
     * The arm travels to {@link Intake#DEPLOY_STOWED_DEG} via MotionMagic.
     *
     * <p>The roller is always stopped here — running the roller while the arm is
     * stowed serves no purpose and could jam the mechanism.
     */
    public void stow() {
        setDeployAngle(Intake.DEPLOY_STOWED_DEG);
        m_roller.setControl(m_neutralReq);
    }

    /**
     * Runs the intake roller at the configured intake duty cycle.
     *
     * <p><b>Guard:</b> the roller only spins if the arm has reached the deployed
     * position ({@link #isDeployed()} returns {@code true}).  If the arm is still
     * in transit or stowed the roller is stopped instead.  This prevents running the
     * roller against the ground or into the robot frame.
     *
     * <p>Call each loop while intaking.  The guard re-evaluates every cycle, so the
     * roller starts automatically once the arm settles into position — no explicit
     * sequencing required by the caller.
     */
    public void runRoller() {
        if (!isDeployed()) {
            m_roller.setControl(m_neutralReq);
            return;
        }
        m_roller.setControl(m_rollerDutyCycleReq.withOutput(Intake.ROLLER_INTAKE_PERCENT));
    }

    /**
     * Runs the roller in reverse to expel a FUEL ball.
     * Does not change the arm position.
     */
    public void exhaust() {
        m_roller.setControl(m_rollerDutyCycleReq.withOutput(Intake.ROLLER_EXHAUST_PERCENT));
    }

    /** Stops the roller without moving the deploy arm. */
    public void stopRoller() {
        m_roller.setControl(m_neutralReq);
    }

    // -------------------------------------------------------------------------
    // Telemetry Accessors
    // -------------------------------------------------------------------------

    /**
     * Returns the current deploy arm position in mechanism degrees.
     *
     * @return Measured arm angle (0° = stowed, {@link Intake#DEPLOY_DEPLOYED_DEG} = deployed).
     */
    public double getDeployAngleDeg() {
        return Units.rotationsToDegrees(
                m_deployLeft.getPosition().getValueAsDouble() / Intake.DEPLOY_GEAR_RATIO);
    }

    /**
     * Returns whether the intake arm has reached the deployed position.
     *
     * @return {@code true} if within {@link Intake#DEPLOY_TOLERANCE_DEG} of
     *         {@link Intake#DEPLOY_DEPLOYED_DEG}.  The intake is mechanically heavy
     *         and carries a spring-loaded flap, so MotionMagic settles within a few
     *         degrees rather than exactly on the setpoint.
     */
    public boolean isDeployed() {
        return Math.abs(getDeployAngleDeg() - Intake.DEPLOY_DEPLOYED_DEG)
                < Intake.DEPLOY_TOLERANCE_DEG;
    }

    /**
     * Returns whether the intake arm has reached the stowed position.
     *
     * <p>Uses a wider tolerance than {@link #isDeployed()} because stowing fights
     * both gravity and the resistance of the open intake flap; the arm may not
     * reach exactly 0° before the controller is satisfied.
     *
     * @return {@code true} if within {@link Intake#STOW_TOLERANCE_DEG} of
     *         {@link Intake#DEPLOY_STOWED_DEG}.
     */
    public boolean isStowed() {
        return Math.abs(getDeployAngleDeg() - Intake.DEPLOY_STOWED_DEG)
                < Intake.STOW_TOLERANCE_DEG;
    }

    // -------------------------------------------------------------------------
    // SysId Characterization
    // -------------------------------------------------------------------------

    /**
     * Returns a quasistatic SysId command for intake deploy arm characterization.
     * The follower motor tracks the master automatically during characterization.
     *
     * @param direction Forward (deploy direction) or reverse (stow direction).
     * @return The SysId quasistatic command.
     */
    public Command sysIdDeployQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineDeploy.quasistatic(direction);
    }

    /**
     * Returns a dynamic SysId command for intake deploy arm characterization.
     *
     * @param direction Forward or reverse step direction.
     * @return The SysId dynamic command.
     */
    public Command sysIdDeployDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineDeploy.dynamic(direction);
    }

    /** @return The deploy {@link SysIdRoutine} for use in a mechanism SysId chooser. */
    public SysIdRoutine getSysIdDeployRoutine() { return m_sysIdRoutineDeploy; }

    // -------------------------------------------------------------------------
    // Fault Detection
    // -------------------------------------------------------------------------

    /**
     * Returns {@code true} if any critical hardware fault is active on any intake motor
     * (deploy left, deploy right, or roller).
     *
     * @return {@code true} if a critical fault is present.
     */
    public boolean hasCriticalFault() {
        return m_deployLeft.getFault_Hardware().getValue()
            || m_deployLeft.getFault_BootDuringEnable().getValue()
            || m_deployLeft.getFault_DeviceTemp().getValue()
            || m_deployRight.getFault_Hardware().getValue()
            || m_deployRight.getFault_BootDuringEnable().getValue()
            || m_deployRight.getFault_DeviceTemp().getValue()
            || m_roller.getFault_Hardware().getValue()
            || m_roller.getFault_BootDuringEnable().getValue()
            || m_roller.getFault_DeviceTemp().getValue();
    }

    // -------------------------------------------------------------------------
    // Private Helpers
    // -------------------------------------------------------------------------

    /**
     * Commands the deploy arm to {@code targetDeg} degrees from vertical (stowed = 0°).
     *
     * <h3>Gravity feedforward</h3>
     * <p>The arm is vertical at 0° (stowed) and ~47.5° from vertical when deployed.
     * Gravity torque on the arm is proportional to {@code sin(angle)}, so the
     * feedforward applied is:
     * <pre>  FF = DEPLOY_KG × sin(currentArmAngle)</pre>
     * Phoenix 6's built-in {@code Arm_Cosine} gravity type is <em>not</em> used
     * (it assumes 0° = horizontal, which is wrong for this arm).
     *
     * <h3>Stow boost</h3>
     * <p>When stowing (lifting against gravity), {@link Intake#DEPLOY_KG_STOW_EXTRA_V}
     * is added on top of the gravity feedforward to give the motors enough torque
     * to start the upward motion and lift the roller assembly.
     *
     * <h3>Asymmetric cruise velocity</h3>
     * <p>Deploying uses a slow cruise velocity (gravity assists; arm would overshoot
     * at high speed). Stowing uses a faster cruise velocity (motors need time to
     * build peak torque against gravity). The MotionMagic config is updated via the
     * non-blocking {@code apply(config, 0.0)} call whenever the direction changes.
     */
    private void setDeployAngle(double targetDeg) {
        double motorRot = Units.degreesToRotations(targetDeg) * Intake.DEPLOY_GEAR_RATIO;
        double armDeg   = getDeployAngleDeg();

        // Determine motion direction (hysteresis dead band prevents flip-flopping at the
        // boundary).  Uses DEPLOY_TOLERANCE_DEG so the same physical threshold governs
        // both position acceptance and direction detection.
        boolean newIsStowing = targetDeg < armDeg - Intake.DEPLOY_TOLERANCE_DEG;

        // Reconfigure the Motion Magic cruise velocity only when direction changes.
        // apply(config, 0.0) is non-blocking (best-effort, no latency stall).
        if (newIsStowing != m_isStowing) {
            m_isStowing = newIsStowing;
            var mmConfig = new MotionMagicConfigs();
            mmConfig.MotionMagicCruiseVelocity = m_isStowing
                    ? Intake.DEPLOY_MM_CRUISE_VEL_STOW_RPS
                    : Intake.DEPLOY_MM_CRUISE_VEL_DEPLOY_RPS;
            mmConfig.MotionMagicAcceleration = Intake.DEPLOY_MM_ACCEL_RPSS;
            mmConfig.MotionMagicJerk         = Intake.DEPLOY_MM_JERK_RPSS2;
            m_deployLeft.getConfigurator().apply(mmConfig, 0.0);
        }

        // sin-based gravity compensation: peak at 90°, zero at 0° (vertical/stowed).
        double gravityFF = Intake.DEPLOY_KG * Math.sin(Math.toRadians(armDeg));

        // Extra lift boost when fighting gravity (stowing direction only).
        double stowBoost = m_isStowing ? Intake.DEPLOY_KG_STOW_EXTRA_V : 0.0;

        m_deployLeft.setControl(
                m_deployPositionReq.withPosition(motorRot)
                                   .withFeedForward(gravityFF + stowBoost));
        // The right motor maintains its follower request continuously in periodic().
    }

    @Override
    public void periodic() {
        // Keep the follower request active every loop cycle.
        m_deployRight.setControl(m_followerReq);
    }

    // -------------------------------------------------------------------------
    // Motor Configuration
    // -------------------------------------------------------------------------

    private void configureDeployMotors() {
        // --- Master (left) ---
        var masterConfig = new TalonFXConfiguration();
        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterConfig.MotorOutput.Inverted    = Intake.DEPLOY_LEFT_INVERT;

        var slot0 = new Slot0Configs();
        slot0.kP = Intake.DEPLOY_KP;
        slot0.kI = Intake.DEPLOY_KI;
        slot0.kD = Intake.DEPLOY_KD;
        slot0.kV = Intake.DEPLOY_KV;
        slot0.kS = Intake.DEPLOY_KS;
        slot0.kA = Intake.DEPLOY_KA;
        // kG and GravityType are intentionally NOT set here.
        // Gravity feedforward is applied manually in setDeployAngle() as
        // kG * sin(armAngle), which is correct for a vertically-stowed arm.
        // CTRE's Arm_Cosine (kG * cos) is incorrect for this arm geometry.
        masterConfig.Slot0 = slot0;

        // Initial MM config uses the deploy (slow) cruise velocity.
        // setDeployAngle() switches to the stow profile when lifting.
        var mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = Intake.DEPLOY_MM_CRUISE_VEL_DEPLOY_RPS;
        mm.MotionMagicAcceleration   = Intake.DEPLOY_MM_ACCEL_RPSS;
        mm.MotionMagicJerk           = Intake.DEPLOY_MM_JERK_RPSS2;
        masterConfig.MotionMagic = mm;

        // Soft limits protect the arm at stow (0°) and deployed (47.5°) hard stops.
        var softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable    = true;
        softLimits.ForwardSoftLimitThreshold =
                Units.degreesToRotations(Intake.DEPLOY_DEPLOYED_DEG) * Intake.DEPLOY_GEAR_RATIO;
        softLimits.ReverseSoftLimitEnable    = true;
        softLimits.ReverseSoftLimitThreshold =
                Units.degreesToRotations(Intake.DEPLOY_STOWED_DEG) * Intake.DEPLOY_GEAR_RATIO;
        masterConfig.SoftwareLimitSwitch = softLimits;

        var masterCurrentLimits = new CurrentLimitsConfigs();
        masterCurrentLimits.StatorCurrentLimit       = Intake.DEPLOY_LEFT_STATOR_LIMIT_A;
        masterCurrentLimits.StatorCurrentLimitEnable = true;
        masterCurrentLimits.SupplyCurrentLimit       = Intake.DEPLOY_LEFT_SUPPLY_LIMIT_A;
        masterCurrentLimits.SupplyCurrentLimitEnable = true;
        masterConfig.CurrentLimits = masterCurrentLimits;

        m_deployLeft.getConfigurator().apply(masterConfig);

        // --- Follower (right) — carries roller motor weight; higher current limits ---
        // Motion is commanded by the master via the Follower control request.
        // DEPLOY_RIGHT_INVERT is configured here for safe standalone operation,
        // but has no effect while the motor is in Follower mode (Phoenix 6 ignores it).
        // Direction in follower mode is controlled by DEPLOY_FOLLOWER_OPPOSES_MASTER.
        // The roller motor is mounted on the right side of the flap, so this motor
        // bears extra load during stow and receives a higher current ceiling.
        var followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.MotorOutput.Inverted    = Intake.DEPLOY_RIGHT_INVERT;

        var followerCurrentLimits = new CurrentLimitsConfigs();
        followerCurrentLimits.StatorCurrentLimit       = Intake.DEPLOY_RIGHT_STATOR_LIMIT_A;
        followerCurrentLimits.StatorCurrentLimitEnable = true;
        followerCurrentLimits.SupplyCurrentLimit       = Intake.DEPLOY_RIGHT_SUPPLY_LIMIT_A;
        followerCurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.CurrentLimits = followerCurrentLimits;

        m_deployRight.getConfigurator().apply(followerConfig);
    }

    private void configureRollerMotor() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit       = Intake.ROLLER_STATOR_LIMIT_A;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit       = Intake.ROLLER_SUPPLY_LIMIT_A;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        m_roller.getConfigurator().apply(config);
    }
}
