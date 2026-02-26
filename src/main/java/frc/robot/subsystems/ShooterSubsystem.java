package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.Shooter;

/**
 * Manages the flywheel and hood mechanisms of the shooting system.
 *
 * <p>The <b>flywheel</b> (Kraken X60, {@link Shooter#FLYWHEEL_CAN_ID}) uses
 * closed-loop velocity control (Slot 0, {@link VelocityVoltage}).
 *
 * <p>The <b>hood</b> (Kraken X60, {@link Shooter#HOOD_CAN_ID}) uses
 * MotionMagic position control (Slot 0, {@link MotionMagicVoltage}) with
 * software limits to protect the mechanical hard stops at
 * [{@link Shooter#HOOD_MIN_ANGLE_DEG}, {@link Shooter#HOOD_MAX_ANGLE_DEG}].
 */
public class ShooterSubsystem extends SubsystemBase {

    // Hardware
    private static final CANBus kCANivore = new CANBus("canivore");
    private final TalonFX m_flywheel = new TalonFX(Shooter.FLYWHEEL_CAN_ID, kCANivore);
    private final TalonFX m_hood     = new TalonFX(Shooter.HOOD_CAN_ID,     kCANivore);

    // Control requests (reused each loop to avoid object allocation)
    private final VelocityVoltage    m_flywheelVelocityReq = new VelocityVoltage(0).withSlot(0);
    private final MotionMagicVoltage m_hoodPositionReq     = new MotionMagicVoltage(0).withSlot(0);
    private final NeutralOut         m_neutralReq          = new NeutralOut();
    private final VoltageOut         m_voltageReq          = new VoltageOut(0);

    /** Last commanded flywheel target in RPM; used for readiness checks. */
    private double m_targetFlywheelRPM = 0.0;

    /** Last commanded hood target in degrees; used for readiness checks. */
    private double m_targetHoodAngleDeg = 0.0;

    // SysId routine for flywheel velocity characterization
    private final SysIdRoutine m_sysIdRoutineFlywheel = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // default ramp rate (1 V/s)
            Volts.of(7), // dynamic step voltage
            null,        // default timeout (10 s)
            state -> SignalLogger.writeString("SysIdFlywheel_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> m_flywheel.setControl(m_voltageReq.withOutput(volts.in(Volts))),
            null,
            this
        )
    );

    // SysId routine for hood position characterization
    // Soft limits remain active so the hood cannot be driven past its hard stops.
    private final SysIdRoutine m_sysIdRoutineHood = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // default ramp rate (1 V/s)
            Volts.of(4), // lower step voltage — position mechanism, limited range
            null,        // default timeout (10 s)
            state -> SignalLogger.writeString("SysIdHood_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> m_hood.setControl(m_voltageReq.withOutput(volts.in(Volts))),
            null,
            this
        )
    );

    /**
     * Constructs the ShooterSubsystem and applies all TalonFX configuration.
     * Must be instantiated once, inside {@link frc.robot.RobotContainer}.
     */
    public ShooterSubsystem() {
        configureFlywheelMotor();
        configureHoodMotor();
    }

    // -------------------------------------------------------------------------
    // Public Command Methods
    // -------------------------------------------------------------------------

    /**
     * Commands the flywheel to the given speed using closed-loop velocity control.
     *
     * @param rpm Target flywheel speed in rotations per minute.
     */
    public void setFlywheelRPM(double rpm) {
        m_targetFlywheelRPM = rpm;
        m_flywheel.setControl(m_flywheelVelocityReq.withVelocity(rpm / 60.0));
    }

    /**
     * Commands the hood to the given angle using MotionMagic position control.
     * The angle is clamped to the configured mechanical limits before being sent.
     *
     * @param angleDeg Target hood angle in mechanism degrees
     *                 (0° = most horizontal, higher = more lofted).
     */
    public void setHoodAngle(double angleDeg) {
        m_targetHoodAngleDeg = Math.max(Shooter.HOOD_MIN_ANGLE_DEG,
                                        Math.min(Shooter.HOOD_MAX_ANGLE_DEG, angleDeg));
        double motorRot = Units.degreesToRotations(m_targetHoodAngleDeg) * Shooter.HOOD_GEAR_RATIO;
        m_hood.setControl(m_hoodPositionReq.withPosition(motorRot));
    }

    /** Stops flywheel output (coast). */
    public void stopFlywheel() {
        m_flywheel.setControl(m_neutralReq);
    }

    /** Stops hood output (brake mode holds last position). */
    public void stopHood() {
        m_hood.setControl(m_neutralReq);
    }

    // -------------------------------------------------------------------------
    // Readiness Queries
    // -------------------------------------------------------------------------

    /**
     * Returns whether the flywheel is within tolerance of its last commanded RPM.
     *
     * @return {@code true} if speed error is ≤ {@link Shooter#FLYWHEEL_TOLERANCE_RPS}.
     */
    public boolean isFlywheelAtSpeed() {
        double errorRPS = Math.abs(m_flywheel.getVelocity().getValueAsDouble()
                - m_targetFlywheelRPM / 60.0);
        return errorRPS <= Shooter.FLYWHEEL_TOLERANCE_RPS;
    }

    /**
     * Returns whether the hood is within tolerance of its last commanded angle.
     *
     * @return {@code true} if angle error is ≤ {@link Shooter#HOOD_TOLERANCE_DEG}.
     */
    public boolean isHoodAtAngle() {
        return Math.abs(getHoodAngleDeg() - m_targetHoodAngleDeg) <= Shooter.HOOD_TOLERANCE_DEG;
    }

    // -------------------------------------------------------------------------
    // Telemetry Accessors
    // -------------------------------------------------------------------------

    /**
     * Returns the current flywheel speed in rotations per minute.
     *
     * @return Measured flywheel speed in RPM.
     */
    public double getFlywheelRPM() {
        return m_flywheel.getVelocity().getValueAsDouble() * 60.0;
    }

    /**
     * Returns the current hood position in mechanism degrees.
     *
     * @return Measured hood angle in degrees.
     */
    public double getHoodAngleDeg() {
        return Units.rotationsToDegrees(m_hood.getPosition().getValueAsDouble()
                / Shooter.HOOD_GEAR_RATIO);
    }

    /** @return Last flywheel target in RPM. */
    public double getTargetFlywheelRPM()  { return m_targetFlywheelRPM; }

    /** @return Last hood target in degrees. */
    public double getTargetHoodAngleDeg() { return m_targetHoodAngleDeg; }

    /**
     * Returns the flywheel motor stator current in amps.
     * Useful for telemetry and SysId log verification.
     *
     * @return Stator current in amps.
     */
    public double getFlywheelStatorCurrentAmps() {
        return m_flywheel.getStatorCurrent().getValueAsDouble();
    }

    // -------------------------------------------------------------------------
    // Fault Detection
    // -------------------------------------------------------------------------

    /**
     * Returns {@code true} if any critical hardware fault is active on the flywheel
     * or hood motor.  Critical faults include hardware failure, boot-during-enable,
     * and over-temperature conditions.
     *
     * <p>Poll this each loop from {@link frc.robot.subsystems.FaultMonitor}.
     *
     * @return {@code true} if a critical fault is present.
     */
    public boolean hasCriticalFault() {
        return m_flywheel.getFault_Hardware().getValue()
            || m_flywheel.getFault_BootDuringEnable().getValue()
            || m_flywheel.getFault_DeviceTemp().getValue()
            || m_hood.getFault_Hardware().getValue()
            || m_hood.getFault_BootDuringEnable().getValue()
            || m_hood.getFault_DeviceTemp().getValue();
    }

    // -------------------------------------------------------------------------
    // SysId Characterization
    // -------------------------------------------------------------------------

    /**
     * Returns a quasistatic SysId command for flywheel velocity characterization.
     * Bind to a controller button in {@link frc.robot.RobotContainer} for field
     * characterization sessions.
     *
     * @param direction Forward or reverse sweep direction.
     * @return The SysId quasistatic command.
     */
    public Command sysIdFlywheelQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineFlywheel.quasistatic(direction);
    }

    /**
     * Returns a dynamic SysId command for flywheel velocity characterization.
     *
     * @param direction Forward or reverse step direction.
     * @return The SysId dynamic command.
     */
    public Command sysIdFlywheelDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineFlywheel.dynamic(direction);
    }

    /**
     * Returns a quasistatic SysId command for hood position characterization.
     *
     * @param direction Forward (increasing angle) or reverse sweep direction.
     * @return The SysId quasistatic command.
     */
    public Command sysIdHoodQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineHood.quasistatic(direction);
    }

    /**
     * Returns a dynamic SysId command for hood position characterization.
     *
     * @param direction Forward or reverse step direction.
     * @return The SysId dynamic command.
     */
    public Command sysIdHoodDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineHood.dynamic(direction);
    }

    /** @return The flywheel {@link SysIdRoutine} for use in a mechanism SysId chooser. */
    public SysIdRoutine getSysIdFlywheelRoutine() { return m_sysIdRoutineFlywheel; }

    /** @return The hood {@link SysIdRoutine} for use in a mechanism SysId chooser. */
    public SysIdRoutine getSysIdHoodRoutine() { return m_sysIdRoutineHood; }

    // -------------------------------------------------------------------------
    // Motor Configuration
    // -------------------------------------------------------------------------

    private void configureFlywheelMotor() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        var slot0 = new Slot0Configs();
        slot0.kP = Shooter.FLYWHEEL_KP;
        slot0.kI = Shooter.FLYWHEEL_KI;
        slot0.kD = Shooter.FLYWHEEL_KD;
        slot0.kV = Shooter.FLYWHEEL_KV;
        slot0.kS = Shooter.FLYWHEEL_KS;
        slot0.kA = Shooter.FLYWHEEL_KA;
        config.Slot0 = slot0;

        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit       = Shooter.FLYWHEEL_STATOR_LIMIT_A;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit       = Shooter.FLYWHEEL_SUPPLY_LIMIT_A;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        m_flywheel.getConfigurator().apply(config);
    }

    private void configureHoodMotor() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var slot0 = new Slot0Configs();
        slot0.kP = Shooter.HOOD_KP;
        slot0.kI = Shooter.HOOD_KI;
        slot0.kD = Shooter.HOOD_KD;
        slot0.kV = Shooter.HOOD_KV;
        slot0.kS = Shooter.HOOD_KS;
        slot0.kA = Shooter.HOOD_KA;
        config.Slot0 = slot0;

        var mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = Shooter.HOOD_MM_CRUISE_VEL_RPS;
        mm.MotionMagicAcceleration   = Shooter.HOOD_MM_ACCEL_RPSS;
        mm.MotionMagicJerk           = Shooter.HOOD_MM_JERK_RPSS2;
        config.MotionMagic = mm;

        // Software limits protect mechanical hard stops.
        var softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable    = true;
        softLimits.ForwardSoftLimitThreshold =
                Units.degreesToRotations(Shooter.HOOD_MAX_ANGLE_DEG) * Shooter.HOOD_GEAR_RATIO;
        softLimits.ReverseSoftLimitEnable    = true;
        softLimits.ReverseSoftLimitThreshold =
                Units.degreesToRotations(Shooter.HOOD_MIN_ANGLE_DEG) * Shooter.HOOD_GEAR_RATIO;
        config.SoftwareLimitSwitch = softLimits;

        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit       = Shooter.HOOD_STATOR_LIMIT_A;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit       = Shooter.HOOD_SUPPLY_LIMIT_A;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        m_hood.getConfigurator().apply(config);
    }
}
