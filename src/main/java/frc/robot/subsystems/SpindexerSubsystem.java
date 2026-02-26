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

import frc.robot.Constants.Spindexer;

/**
 * Controls the spindexer (spinning indexer disk) that queues FUEL balls and
 * presents them to the feeder.
 *
 * <p>The spindexer (Kraken X60, {@link Spindexer#SPINDEXER_CAN_ID}) runs on
 * open-loop duty-cycle control.  Stator current is monitored and exposed to
 * the {@link frc.robot.superstructure.Superstructure} for jam detection.
 */
public class SpindexerSubsystem extends SubsystemBase {

    private final TalonFX m_spindexer = new TalonFX(Spindexer.SPINDEXER_CAN_ID);

    private final DutyCycleOut m_dutyCycleReq = new DutyCycleOut(0);
    private final NeutralOut   m_neutralReq   = new NeutralOut();
    private final VoltageOut   m_voltageReq   = new VoltageOut(0);

    // SysId routine for spindexer velocity characterization
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // default ramp rate (1 V/s)
            Volts.of(6), // dynamic step voltage
            null,        // default timeout (10 s)
            state -> SignalLogger.writeString("SysIdSpindexer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> m_spindexer.setControl(m_voltageReq.withOutput(volts.in(Volts))),
            null,
            this
        )
    );

    /**
     * Constructs the SpindexerSubsystem and applies motor configuration.
     * Must be instantiated once, inside {@link frc.robot.RobotContainer}.
     */
    public SpindexerSubsystem() {
        configureSpindexerMotor();
    }

    // -------------------------------------------------------------------------
    // Public Command Methods
    // -------------------------------------------------------------------------

    /**
     * Runs the spindexer forward at {@link Spindexer#SPINDEXER_FORWARD_PERCENT}
     * to advance FUEL balls toward the feeder.
     */
    public void run() {
        m_spindexer.setControl(m_dutyCycleReq.withOutput(Spindexer.SPINDEXER_FORWARD_PERCENT));
    }

    /**
     * Runs the spindexer in reverse at {@link Spindexer#SPINDEXER_REVERSE_PERCENT}
     * to clear a jammed ball.
     */
    public void reverse() {
        m_spindexer.setControl(m_dutyCycleReq.withOutput(Spindexer.SPINDEXER_REVERSE_PERCENT));
    }

    /** Stops the spindexer motor immediately. */
    public void stop() {
        m_spindexer.setControl(m_neutralReq);
    }

    // -------------------------------------------------------------------------
    // Telemetry / State
    // -------------------------------------------------------------------------

    /**
     * Returns the current stator current draw of the spindexer motor in amps.
     * Used by {@link frc.robot.superstructure.Superstructure} for jam detection;
     * compare against {@link Spindexer#SPINDEXER_JAM_CURRENT_A}.
     *
     * @return Spindexer stator current in amps.
     */
    public double getStatorCurrentAmps() {
        return m_spindexer.getStatorCurrent().getValueAsDouble();
    }

    // -------------------------------------------------------------------------
    // SysId Characterization
    // -------------------------------------------------------------------------

    /**
     * Returns a quasistatic SysId command for spindexer velocity characterization.
     *
     * @param direction Forward or reverse sweep direction.
     * @return The SysId quasistatic command.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a dynamic SysId command for spindexer velocity characterization.
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
     * Returns {@code true} if any critical hardware fault is active on the spindexer motor.
     *
     * @return {@code true} if a critical fault is present.
     */
    public boolean hasCriticalFault() {
        return m_spindexer.getFault_Hardware().getValue()
            || m_spindexer.getFault_BootDuringEnable().getValue()
            || m_spindexer.getFault_DeviceTemp().getValue();
    }

    // -------------------------------------------------------------------------
    // Motor Configuration
    // -------------------------------------------------------------------------

    private void configureSpindexerMotor() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit       = Spindexer.SPINDEXER_STATOR_LIMIT_A;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit       = Spindexer.SPINDEXER_SUPPLY_LIMIT_A;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        m_spindexer.getConfigurator().apply(config);
    }
}
