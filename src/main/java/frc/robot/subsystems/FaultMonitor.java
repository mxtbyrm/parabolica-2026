package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Polls all motor subsystems and the CANivore bus each loop for critical faults,
 * surfacing them as WPILib {@link Alert}s on the DriverStation and SmartDashboard.
 *
 * <p>An alert fires when a subsystem reports a hardware fault, over-temperature,
 * or boot-during-enable condition.  The CAN bus alert fires when the bus has
 * entered Bus-Off state or the TX buffer has overflowed since boot.
 *
 * <p>Instantiate once in {@link frc.robot.RobotContainer}.  Because this class
 * extends {@link SubsystemBase}, the CommandScheduler will call {@link #periodic()}
 * automatically every 20 ms without any additional wiring.
 */
public class FaultMonitor extends SubsystemBase {

    // -------------------------------------------------------------------------
    // Hardware references
    // -------------------------------------------------------------------------

    private final ShooterSubsystem   m_shooter;
    private final TurretSubsystem    m_turret;
    private final FeederSubsystem    m_feeder;
    private final SpindexerSubsystem m_spindexer;
    private final IntakeSubsystem    m_intake;

    private final CANBus m_CANivore = new CANBus("CANivore");

    // -------------------------------------------------------------------------
    // Persistent DriverStation alerts
    // -------------------------------------------------------------------------

    private final Alert m_shooterFaultAlert   =
            new Alert("Shooter motor fault (hardware/temp/boot-during-enable)", AlertType.kError);
    private final Alert m_turretFaultAlert    =
            new Alert("Turret motor fault (hardware/temp/boot-during-enable)",  AlertType.kError);
    private final Alert m_feederFaultAlert    =
            new Alert("Feeder motor fault (hardware/temp/boot-during-enable)",  AlertType.kError);
    private final Alert m_spindexerFaultAlert =
            new Alert("Spindexer motor fault (hardware/temp/boot-during-enable)", AlertType.kError);
    private final Alert m_intakeFaultAlert    =
            new Alert("Intake motor fault (hardware/temp/boot-during-enable)",  AlertType.kError);
    private final Alert m_canBusAlert         =
            new Alert("CANivore bus error — check CAN wiring (BusOff or TX overflow)", AlertType.kWarning);

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * Constructs the FaultMonitor.
     *
     * @param shooter   The shooter subsystem.
     * @param turret    The turret subsystem.
     * @param feeder    The feeder subsystem.
     * @param spindexer The spindexer subsystem.
     * @param intake    The intake subsystem.
     */
    public FaultMonitor(
            ShooterSubsystem   shooter,
            TurretSubsystem    turret,
            FeederSubsystem    feeder,
            SpindexerSubsystem spindexer,
            IntakeSubsystem    intake) {
        m_shooter   = shooter;
        m_turret    = turret;
        m_feeder    = feeder;
        m_spindexer = spindexer;
        m_intake    = intake;
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Motor-level faults — set(true) activates the alert on DriverStation.
        m_shooterFaultAlert.set(m_shooter.hasCriticalFault());
        m_turretFaultAlert.set(m_turret.hasCriticalFault());
        m_feederFaultAlert.set(m_feeder.hasCriticalFault());
        m_spindexerFaultAlert.set(m_spindexer.hasCriticalFault());
        m_intakeFaultAlert.set(m_intake.hasCriticalFault());

        // CAN bus health — Bus-Off means the controller has stopped transmitting.
        var canStatus = m_CANivore.getStatus();
        m_canBusAlert.set(canStatus.BusOffCount > 0 || canStatus.TxFullCount > 0);
    }

    // -------------------------------------------------------------------------
    // Aggregate Query
    // -------------------------------------------------------------------------

    /**
     * Returns {@code true} if any motor subsystem currently has an active critical fault.
     *
     * <p>The superstructure (or any command) can gate safety-critical actions on this
     * method — for example, inhibiting shooting when a motor reports a hardware error.
     *
     * @return {@code true} if any critical fault is active across all monitored subsystems.
     */
    public boolean hasAnyCriticalFault() {
        return m_shooter.hasCriticalFault()
            || m_turret.hasCriticalFault()
            || m_feeder.hasCriticalFault()
            || m_spindexer.hasCriticalFault()
            || m_intake.hasCriticalFault();
    }
}
