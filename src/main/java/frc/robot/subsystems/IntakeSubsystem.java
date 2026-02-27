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
import com.ctre.phoenix6.signals.GravityTypeValue;
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
 * {@link Intake#DEPLOY_RIGHT_OPPOSES_MASTER}{@code = true} so both produce the same
 * physical arm motion despite their mirrored orientations.
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

    // Hardware
    private final TalonFX m_deployLeft  = new TalonFX(Intake.DEPLOY_LEFT_CAN_ID);
    private final TalonFX m_deployRight = new TalonFX(Intake.DEPLOY_RIGHT_CAN_ID);
    private final TalonFX m_roller      = new TalonFX(Intake.ROLLER_CAN_ID);

    // Control requests
    private final MotionMagicVoltage m_deployPositionReq =
            new MotionMagicVoltage(0).withSlot(0);
    // opposeDirection = false: inversion is handled by the motor's own MotorOutputConfigs.Inverted
    // (InvertedValue.Clockwise_Positive on the right motor), not by the Follower request.
    private final Follower m_followerReq =
            new Follower(Intake.DEPLOY_LEFT_CAN_ID, Intake.DEPLOY_RIGHT_OPPOSES_MASTER);
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
    }

    // -------------------------------------------------------------------------
    // Public Command Methods
    // -------------------------------------------------------------------------

    /**
     * Deploys the intake arm to the ground-contact position and starts the roller.
     * The arm moves to {@link Intake#DEPLOY_DEPLOYED_DEG} via MotionMagic.
     */
    public void deploy() {
        setDeployAngle(Intake.DEPLOY_DEPLOYED_DEG);
        m_roller.setControl(m_rollerDutyCycleReq.withOutput(Intake.ROLLER_INTAKE_PERCENT));
    }

    /**
     * Stows the intake arm to the retracted position and stops the roller.
     * The arm moves to {@link Intake#DEPLOY_STOWED_DEG} via MotionMagic.
     */
    public void stow() {
        setDeployAngle(Intake.DEPLOY_STOWED_DEG);
        m_roller.setControl(m_neutralReq);
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
     * Returns whether the intake arm is at or near the deployed position.
     *
     * @return {@code true} if within 2 degrees of {@link Intake#DEPLOY_DEPLOYED_DEG}.
     */
    public boolean isDeployed() {
        return Math.abs(getDeployAngleDeg() - Intake.DEPLOY_DEPLOYED_DEG) < 2.0;
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

    private void setDeployAngle(double angleDeg) {
        double motorRot = Units.degreesToRotations(angleDeg) * Intake.DEPLOY_GEAR_RATIO;
        m_deployLeft.setControl(m_deployPositionReq.withPosition(motorRot));
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
        slot0.kG = Intake.DEPLOY_KG;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        masterConfig.Slot0 = slot0;

        var mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = Intake.DEPLOY_MM_CRUISE_VEL_RPS;
        mm.MotionMagicAcceleration   = Intake.DEPLOY_MM_ACCEL_RPSS;
        mm.MotionMagicJerk           = Intake.DEPLOY_MM_JERK_RPSS2;
        masterConfig.MotionMagic = mm;

        // Soft limits protect mechanical hard stops at stow (0°) and fully deployed (90°).
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
        // Motion is commanded by the master; the follower replicates the master's
        // output in the opposite direction (DEPLOY_RIGHT_OPPOSES_MASTER = true).
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
