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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.Intake;

/**
 * Manages the ground intake mechanism, which consists of:
 *
 * <ul>
 *   <li><b>Deploy rack-and-pinion</b> — master (left, {@link Intake#DEPLOY_LEFT_CAN_ID}) and
 *       follower (right, {@link Intake#DEPLOY_RIGHT_CAN_ID}) Kraken X60 motors using
 *       MotionMagic position control to extend/retract the intake via a rack-and-pinion
 *       linkage.  Positions are expressed in motor rotations.</li>
 *   <li><b>Roller</b> — one Kraken X60 ({@link Intake#ROLLER_CAN_ID}) running
 *       open-loop duty-cycle to ingest or expel FUEL.</li>
 * </ul>
 *
 * <h2>Physical Layout</h2>
 * <p>The master motor (left) drives the pinion; the right motor follows via
 * {@code opposeDirection = true} ({@link Intake#DEPLOY_FOLLOWER_OPPOSES_MASTER}).
 * Phoenix 6 ignores {@code MotorOutputConfigs.Inverted} in Follower mode; direction
 * is controlled solely by the {@code opposeDirection} flag.
 *
 * <h2>No Gravity Feedforward</h2>
 * <p>Unlike the previous pivot-arm design, a rack-and-pinion mechanism does not
 * require angle-dependent gravity compensation.  MotionMagic PID alone is sufficient.
 */
public class IntakeSubsystem extends SubsystemBase {

    // ── Roller state tracking ─────────────────────────────────────────────────
    private boolean m_rollerRunning = false;

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
        // Seed the master encoder at the stowed position on boot.
        // Start the robot with the rack fully retracted so the zero reference is correct.
        m_deployLeft.setPosition(Intake.DEPLOY_STOWED_ROT);
    }

    // -------------------------------------------------------------------------
    // Public Command Methods
    // -------------------------------------------------------------------------

    /**
     * Extends the rack to the fully deployed (ground-contact) position.
     * Does <em>not</em> start the roller — call {@link #runRoller()} separately.
     */
    public void deploy() {
        setDeployPosition(Intake.DEPLOY_DEPLOYED_ROT);
    }

    /**
     * Retracts the rack to the fully stowed position.
     * The roller is <em>not</em> touched — deploy and roller are fully independent.
     */
    public void stow() {
        setDeployPosition(Intake.DEPLOY_STOWED_ROT);
    }

    /**
     * Retracts the rack to the intermediate agitate position (between deployed and stowed).
     * Used by the agitate command to jostle balls through the intake.
     * The roller is not affected.
     */
    public void agitate() {
        setDeployPosition(Intake.DEPLOY_AGITATE_ROT);
    }

    /**
     * Runs the intake roller at the configured intake duty cycle.
     *
     * <p>The roller is fully independent of the deploy arm — the operator
     * controls each mechanism separately.  No arm-position guard is applied.
     */
    public void runRoller() {
        m_rollerRunning = true;
        m_roller.setControl(m_rollerDutyCycleReq.withOutput(Intake.ROLLER_INTAKE_PERCENT));
    }

    /**
     * Runs the intake roller at a custom duty cycle percentage.
     * Useful for low-speed agitation without full intake power.
     *
     * @param percent Duty cycle output (0.0 – 1.0 forward, negative for reverse).
     */
    public void runRollerAt(double percent) {
        m_rollerRunning = true;
        m_roller.setControl(m_rollerDutyCycleReq.withOutput(percent));
    }

    /**
     * Runs the roller in reverse to expel a FUEL ball.
     * Does not change the arm position.
     */
    public void exhaust() {
        m_rollerRunning = true;
        m_roller.setControl(m_rollerDutyCycleReq.withOutput(Intake.ROLLER_EXHAUST_PERCENT));
    }

    /** Stops the roller without moving the deploy arm. */
    public void stopRoller() {
        m_rollerRunning = false;
        m_roller.setControl(m_neutralReq);
    }

    /**
     * Returns {@code true} while the roller motor is actively commanded to spin
     * (intake or exhaust direction). {@code false} after {@link #stopRoller()}.
     */
    public boolean isRollerRunning() {
        return m_rollerRunning;
    }

    // -------------------------------------------------------------------------
    // Telemetry Accessors
    // -------------------------------------------------------------------------

    /**
     * Returns the current deploy rack position in motor rotations.
     *
     * @return Measured motor position (0 = stowed, {@link Intake#DEPLOY_DEPLOYED_ROT} = deployed).
     */
    public double getDeployPositionRot() {
        return m_deployLeft.getPosition().getValueAsDouble();
    }

    /** Returns {@code true} if the rack is within tolerance of the fully deployed position. */
    public boolean isDeployed() {
        return Math.abs(getDeployPositionRot() - Intake.DEPLOY_DEPLOYED_ROT)
                < Intake.DEPLOY_TOLERANCE_ROT;
    }

    /** Returns {@code true} if the rack is within tolerance of the fully stowed position. */
    public boolean isStowed() {
        return Math.abs(getDeployPositionRot() - Intake.DEPLOY_STOWED_ROT)
                < Intake.STOW_TOLERANCE_ROT;
    }

    /** Returns {@code true} if the rack is within tolerance of the agitate (intermediate) position. */
    public boolean isAtAgitatePosition() {
        return Math.abs(getDeployPositionRot() - Intake.DEPLOY_AGITATE_ROT)
                < Intake.AGITATE_TOLERANCE_ROT;
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
     * Commands the rack to {@code targetRot} motor rotations via MotionMagic.
     *
     * <p>No gravity feedforward is needed for the rack-and-pinion mechanism.
     * Motion is symmetric in both directions — a single cruise velocity profile applies.
     */
    private void setDeployPosition(double targetRot) {
        m_deployLeft.setControl(m_deployPositionReq.withPosition(targetRot));
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
        // kG is not set — no gravity feedforward needed for rack-and-pinion.
        masterConfig.Slot0 = slot0;

        // Single symmetric MM profile — rack-and-pinion needs no asymmetric cruise velocity.
        var mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = Intake.DEPLOY_MM_CRUISE_VEL_RPS;
        mm.MotionMagicAcceleration   = Intake.DEPLOY_MM_ACCEL_RPSS;
        mm.MotionMagicJerk           = Intake.DEPLOY_MM_JERK_RPSS2;
        masterConfig.MotionMagic = mm;

        // Soft limits protect the rack at its mechanical travel endpoints.
        var softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable    = true;
        softLimits.ForwardSoftLimitThreshold = Intake.DEPLOY_DEPLOYED_ROT;
        softLimits.ReverseSoftLimitEnable    = true;
        softLimits.ReverseSoftLimitThreshold = Intake.DEPLOY_STOWED_ROT;
        masterConfig.SoftwareLimitSwitch = softLimits;

        var masterCurrentLimits = new CurrentLimitsConfigs();
        masterCurrentLimits.StatorCurrentLimit       = Intake.DEPLOY_LEFT_STATOR_LIMIT_A;
        masterCurrentLimits.StatorCurrentLimitEnable = true;
        masterCurrentLimits.SupplyCurrentLimit       = Intake.DEPLOY_LEFT_SUPPLY_LIMIT_A;
        masterCurrentLimits.SupplyCurrentLimitEnable = true;
        masterConfig.CurrentLimits = masterCurrentLimits;

        m_deployLeft.getConfigurator().apply(masterConfig);

        // --- Follower (right) ---
        // Motion is commanded by the master via the Follower control request.
        // DEPLOY_RIGHT_INVERT is configured here for safe standalone operation,
        // but has no effect while the motor is in Follower mode (Phoenix 6 ignores it).
        // Direction in follower mode is controlled by DEPLOY_FOLLOWER_OPPOSES_MASTER.
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
