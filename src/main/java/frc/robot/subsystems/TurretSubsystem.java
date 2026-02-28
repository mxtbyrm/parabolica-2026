package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.Turret;

/**
 * Controls the turret rotation mechanism powered by a Kraken X44 motor.
 *
 * <p>The turret uses MotionMagic position control (Slot 0) with symmetric software
 * limits at ±{@link Turret#TURRET_FORWARD_LIMIT_DEG} degrees to prevent cable wrap.
 * Zero degrees represents the robot's forward-facing direction; positive angles are
 * counter-clockwise when viewed from above.
 *
 * <p><b>Homing note:</b> The turret must be zeroed at the forward position on robot
 * power-up (or via a limit switch / CANcoder absolute position) before issuing any
 * {@link #setAngle(double)} commands.
 */
public class TurretSubsystem extends SubsystemBase {

    private final TalonFX m_turret = new TalonFX(Turret.TURRET_CAN_ID);

    private final MotionMagicVoltage m_positionReq  = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut       m_dutyCycleReq = new DutyCycleOut(0);
    private final NeutralOut         m_neutralReq   = new NeutralOut();
    private final VoltageOut         m_voltageReq   = new VoltageOut(0);

    // SysId routine for turret position characterization
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // default ramp rate (1 V/s)
            Volts.of(4), // dynamic step voltage — low for a position mechanism
            null,        // default timeout (10 s)
            state -> SignalLogger.writeString("SysIdTurret_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> m_turret.setControl(m_voltageReq.withOutput(volts.in(Volts))),
            null,
            this
        )
    );

    /** Last commanded target angle in degrees; used by {@link #isAligned()}. */
    private double m_targetAngleDeg = 0.0;

    /**
     * Constructs the TurretSubsystem and applies all motor configuration.
     * Must be instantiated once, inside {@link frc.robot.RobotContainer}.
     */
    public TurretSubsystem() {
        configureTurretMotor();
        // Seed the turret encoder to 0° (robot-forward / home position) on boot.
        // The robot is assumed to be aimed straight ahead when first powered on.
        zeroPosition();
    }

    // -------------------------------------------------------------------------
    // Public Command Methods
    // -------------------------------------------------------------------------

    /**
     * Commands the turret to the specified angle using MotionMagic.
     * The angle is silently clamped to the configured soft limits.
     *
     * @param angleDeg Target turret angle in degrees.
     *                 Positive = counter-clockwise from robot forward.
     */
    public void setAngle(double angleDeg) {
        // Normalize to [-180, +180] so the turret always takes the path through
        // center rather than wrapping past the cable limits.
        // e.g. +190° → -170°: motor goes backward through 0° instead of forward past +175°.
        double normalized = ((angleDeg % 360.0) + 540.0) % 360.0 - 180.0;
        m_targetAngleDeg = Math.max(Turret.TURRET_REVERSE_LIMIT_DEG,
                           Math.min(Turret.TURRET_FORWARD_LIMIT_DEG, normalized));
        double motorRot = Units.degreesToRotations(m_targetAngleDeg) * Turret.TURRET_GEAR_RATIO;
        // Spring feedforward: the Vulcan spring applies a restoring force proportional
        // to turret angle.  Feed forward a compensating voltage (positive when CCW,
        // negative when CW) so MotionMagic doesn't have to fight it with error alone.
        double springFF = m_targetAngleDeg * Turret.TURRET_SPRING_KF;
        m_turret.setControl(m_positionReq.withPosition(motorRot).withFeedForward(springFF));
    }

    /**
     * Stops turret output.  The motor will coast to rest.
     * Call this when returning to a state where the turret should not be commanded.
     */
    public void stop() {
        m_turret.setControl(m_neutralReq);
    }

    // -------------------------------------------------------------------------
    // Readiness Queries
    // -------------------------------------------------------------------------

    /**
     * Returns whether the turret is within alignment tolerance of its commanded angle.
     *
     * @return {@code true} if the angular error is ≤ {@link Turret#TURRET_TOLERANCE_DEG}.
     */
    public boolean isAligned() {
        return Math.abs(getAngleDeg() - m_targetAngleDeg) <= Turret.TURRET_TOLERANCE_DEG;
    }

    // -------------------------------------------------------------------------
    // Telemetry Accessors
    // -------------------------------------------------------------------------

    /**
     * Returns the current turret position in mechanism degrees.
     *
     * @return Measured turret angle in degrees (positive = CCW from forward).
     */
    public double getAngleDeg() {
        return Units.rotationsToDegrees(
                m_turret.getPosition().getValueAsDouble() / Turret.TURRET_GEAR_RATIO);
    }

    /** @return Last turret target in degrees. */
    public double getTargetAngleDeg() { return m_targetAngleDeg; }

    /**
     * Returns the turret motor stator current in amps.
     * Used by {@link frc.robot.commands.HomeTurretCommand} for stall detection.
     *
     * @return Stator current in amps.
     */
    public double getStatorCurrentAmps() {
        return m_turret.getStatorCurrent().getValueAsDouble();
    }

    // -------------------------------------------------------------------------
    // Fault Detection
    // -------------------------------------------------------------------------

    /**
     * Returns {@code true} if any critical hardware fault is active on the turret motor.
     *
     * @return {@code true} if a critical fault is present.
     */
    public boolean hasCriticalFault() {
        return m_turret.getFault_Hardware().getValue()
            || m_turret.getFault_BootDuringEnable().getValue()
            || m_turret.getFault_DeviceTemp().getValue();
    }

    // -------------------------------------------------------------------------
    // SysId Characterization
    // -------------------------------------------------------------------------

    /**
     * Returns a quasistatic SysId command for turret position characterization.
     *
     * @param direction Forward (CCW) or reverse (CW) sweep direction.
     * @return The SysId quasistatic command.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a dynamic SysId command for turret position characterization.
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
    // Homing Support
    // -------------------------------------------------------------------------

    /**
     * Runs the turret at the given open-loop duty cycle.
     * Intended only for use during the {@link frc.robot.commands.HomeTurretCommand}
     * homing sequence; do not call during normal operation.
     *
     * @param percent Duty cycle in [-1, 1].  Positive = clockwise.
     */
    public void driveAtPercent(double percent) {
        m_turret.setControl(m_dutyCycleReq.withOutput(percent));
    }

    /**
     * Resets the motor's internal position sensor to zero at the current physical
     * location.  Call this immediately after the turret reaches its home switch or
     * hard stop during the homing routine.
     */
    public void zeroPosition() {
        m_turret.setPosition(0.0);
        m_targetAngleDeg = 0.0;
    }

    // -------------------------------------------------------------------------
    // Motor Configuration
    // -------------------------------------------------------------------------

    private void configureTurretMotor() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var slot0 = new Slot0Configs();
        slot0.kP = Turret.TURRET_KP;
        slot0.kI = Turret.TURRET_KI;
        slot0.kD = Turret.TURRET_KD;
        slot0.kV = Turret.TURRET_KV;
        slot0.kS = Turret.TURRET_KS;
        slot0.kA = Turret.TURRET_KA;
        config.Slot0 = slot0;

        var mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = Turret.TURRET_MM_CRUISE_VEL_RPS;
        mm.MotionMagicAcceleration   = Turret.TURRET_MM_ACCEL_RPSS;
        mm.MotionMagicJerk           = Turret.TURRET_MM_JERK_RPSS2;
        config.MotionMagic = mm;

        var softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable    = true;
        softLimits.ForwardSoftLimitThreshold =
                Units.degreesToRotations(Turret.TURRET_FORWARD_LIMIT_DEG) * Turret.TURRET_GEAR_RATIO;
        softLimits.ReverseSoftLimitEnable    = true;
        softLimits.ReverseSoftLimitThreshold =
                Units.degreesToRotations(Turret.TURRET_REVERSE_LIMIT_DEG) * Turret.TURRET_GEAR_RATIO;
        config.SoftwareLimitSwitch = softLimits;

        var currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit       = Turret.TURRET_STATOR_LIMIT_A;
        currentLimits.StatorCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit       = Turret.TURRET_SUPPLY_LIMIT_A;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        m_turret.getConfigurator().apply(config);
    }
}
