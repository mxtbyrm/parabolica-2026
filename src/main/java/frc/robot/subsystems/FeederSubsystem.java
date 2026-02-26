package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.Feeder;

/**
 * Controls the ball feeder mechanism that transfers FUEL from the spindexer
 * into the shooter flywheel.
 *
 * <p>The feeder (Kraken X60, {@link Feeder#FEEDER_CAN_ID}) runs on open-loop
 * duty-cycle control for simplicity.  Stator current is monitored and exposed
 * to the {@link frc.robot.superstructure.Superstructure} for jam detection.
 */
public class FeederSubsystem extends SubsystemBase {

    private final TalonFX m_feeder = new TalonFX(Feeder.FEEDER_CAN_ID);

    private final DutyCycleOut m_dutyCycleReq = new DutyCycleOut(0);
    private final NeutralOut   m_neutralReq   = new NeutralOut();
    private final VoltageOut   m_voltageReq   = new VoltageOut(0);

    // SysId routine for feeder velocity characterization
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // default ramp rate (1 V/s)
            Volts.of(6), // dynamic step voltage
            null,        // default timeout (10 s)
            state -> SignalLogger.writeString("SysIdFeeder_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> m_feeder.setControl(m_voltageReq.withOutput(volts.in(Volts))),
            null,
            this
        )
    );

    /**
     * Constructs the FeederSubsystem and applies motor configuration.
     * Must be instantiated once, inside {@link frc.robot.RobotContainer}.
     */
    public FeederSubsystem() {
        configureFeederMotor();
    }

    // -------------------------------------------------------------------------
    // Public Command Methods
    // -------------------------------------------------------------------------

    /**
     * Runs the feeder forward at {@link Feeder#FEEDER_FORWARD_PERCENT} to
     * deliver a FUEL ball into the shooter.
     */
    public void feed() {
        m_feeder.setControl(m_dutyCycleReq.withOutput(Feeder.FEEDER_FORWARD_PERCENT));
    }

    /**
     * Runs the feeder in reverse at {@link Feeder#FEEDER_REVERSE_PERCENT} to
     * eject a jammed ball back toward the spindexer.
     */
    public void reverse() {
        m_feeder.setControl(m_dutyCycleReq.withOutput(Feeder.FEEDER_REVERSE_PERCENT));
    }

    /**
     * Stops the feeder motor immediately.
     * Use after each ball is confirmed delivered to prevent double-feeding.
     */
    public void stop() {
        m_feeder.setControl(m_neutralReq);
    }

    // -------------------------------------------------------------------------
    // Telemetry / State
    // -------------------------------------------------------------------------

    /**
     * Returns the current stator current draw of the feeder motor in amps.
     * Used by {@link frc.robot.superstructure.Superstructure} for jam detection;
     * compare against {@link Feeder#FEEDER_JAM_CURRENT_A}.
     *
     * @return Feeder stator current in amps.
     */
    public double getStatorCurrentAmps() {
        return m_feeder.getStatorCurrent().getValueAsDouble();
    }

    // -------------------------------------------------------------------------
    // SysId Characterization
    // -------------------------------------------------------------------------

    /**
     * Returns a quasistatic SysId command for feeder velocity characterization.
     *
     * @param direction Forward or reverse sweep direction.
     * @return The SysId quasistatic command.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a dynamic SysId command for feeder velocity characterization.
     *
     * @param direction Forward or reverse step direction.
     * @return The SysId dynamic command.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    /** @return The raw {@link SysIdRoutine} for use in a mechanism SysId chooser. */
    public SysIdRoutine getSysIdRoutine() { return m_sysIdRoutine; }

    // -------------------------------------------------------------------------
    // Fault Detection
    // -------------------------------------------------------------------------

    /**
     * Returns {@code true} if any critical hardware fault is active on the feeder motor.
     *
     * @return {@code true} if a critical fault is present.
     */
    public boolean hasCriticalFault() {
        return m_feeder.getFault_Hardware().getValue()
            || m_feeder.getFault_BootDuringEnable().getValue()
            || m_feeder.getFault_DeviceTemp().getValue();
    }

    // -------------------------------------------------------------------------
    // Motor Configuration
    // -------------------------------------------------------------------------

    private void configureFeederMotor() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit       = Feeder.FEEDER_STATOR_LIMIT_A;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit       = Feeder.FEEDER_SUPPLY_LIMIT_A;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        m_feeder.getConfigurator().apply(config);
    }
}
