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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // FOC enabled: ~15% more torque and better bandwidth on Kraken X44.
    private final MotionMagicVoltage m_positionReq  = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
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

    /** Last encoder-space angle actually sent to the motor via setControl(). */
    private double m_lastCommandedEncoderDeg = 0.0;

    /**
     * Constructs the TurretSubsystem and applies all motor configuration.
     * Must be instantiated once, inside {@link frc.robot.RobotContainer}.
     */
    public TurretSubsystem() {
        configureTurretMotor();
        // Seed encoder to 0 at the forward-facing boot position (intake and turret aligned).
        // All soft limits and coordinate transforms use this as the fixed reference.
        // Cable-home (spring equilibrium) is at TURRET_CABLE_HOME_ENCODER_DEG, not at 0.
        m_turret.setPosition(0.0);
        m_targetAngleDeg = 0.0; // 0° robot-relative = forward
    }

    // -------------------------------------------------------------------------
    // Public Command Methods
    // -------------------------------------------------------------------------

    /**
     * Commands the turret to the specified angle using MotionMagic, with no
     * robot-rotation feedforward.  Use {@link #setAngle(double, double)} when
     * the robot is rotating so the turret can keep up without lagging.
     *
     * @param angleDeg Target turret angle in degrees.
     *                 Positive = counter-clockwise from robot forward.
     */
    public void setAngle(double angleDeg) {
        setAngle(angleDeg, 0.0);
    }

    /**
     * Commands the turret to the specified angle using MotionMagic, with a
     * full motion-tracking velocity feedforward.
     *
     * <p>In the robot frame, the hub's angular rate changes due to two effects:
     * <ul>
     *   <li><b>Robot rotation (ω):</b> heading changes at ω → hub robot-relative
     *       angle changes at −ω.</li>
     *   <li><b>Robot translation:</b> the turret pivot has lateral velocity
     *       {@code vLateral} (m/s, perpendicular to hub direction) → hub angle
     *       changes at −{@code vLateral/d}.</li>
     * </ul>
     *
     * <p>Total rate of change: {@code d(θ_hub)/dt = −vLateral/d − ω}.
     * Required turret tracking rate (same sign as hub) {@code = −vLateral/d − ω}.
     * Required motor velocity {@code = (vLateral/d + ω) / (2π) × GEAR_RATIO} rps.
     *
     * <p>The caller should pass {@code trackingRateRadPerSec = ω + vLateral/d}
     * where {@code vLateral = −vxT·sin(θ) + vyT·cos(θ)}, with
     * {@code vxT = vx − ω·TY_OFFSET}, {@code vyT = vy + ω·TX_OFFSET},
     * all in the robot frame and θ the hub direction in robot frame.
     *
     * <p>Sign check: robot CCW (+ω), no translation, no offset →
     * trackingRate = ω → motor CCW → turret CW → hub (appearing to move CW) tracked. ✓
     *
     * @param angleDeg              Target turret angle in degrees (0 = forward, positive = CCW).
     * @param trackingRateRadPerSec Combined angular tracking rate: {@code ω + vLateral/d} (rad/s).
     */
    public void setAngle(double angleDeg, double trackingRateRadPerSec) {
        // NOTE: aim trim (TURRET_AIM_TRIM_DEG) is applied by the caller
        // (Superstructure.commandTurretAngle) — do NOT add it here or it
        // would be applied twice on every vision-targeting call.

        // Convert robot-relative angle → encoder angle.
        // Motor CCW (positive encoder) drives the turret physically CW through the gearbox,
        // so the signs are inverted: encoder = -angleDeg (encoder 0 = forward).
        double encoderAngleDeg = -angleDeg;

        // Wrap the requested angle into the turret's physical cable-travel range
        // (REVERSE_LIMIT … FORWARD_LIMIT, a 360° window) so that commands arriving
        // from vision or odometry in an arbitrary frame always map to the
        // reachable side of the cable wrap rather than clamping to a soft limit.
        encoderAngleDeg = MathUtil.inputModulus(
                encoderAngleDeg,
                Turret.TURRET_REVERSE_LIMIT_DEG,
                Turret.TURRET_FORWARD_LIMIT_DEG);

        // Clamp in ENCODER space as a safety net (inputModulus guarantees the
        // value is in range, but belt-and-suspenders is prudent near hardware).
        double clampedEncoderDeg = Math.max(Turret.TURRET_REVERSE_LIMIT_DEG,
                                   Math.min(Turret.TURRET_FORWARD_LIMIT_DEG, encoderAngleDeg));

        // Convert clamped encoder angle back to robot-relative for isAligned() and telemetry.
        m_targetAngleDeg = -clampedEncoderDeg;

        // Setpoint hysteresis: if the turret is already within tolerance AND the new
        // encoder target is very close to the last commanded one, skip issuing a new
        // setControl().  Vision jitter can produce ±0.3° setpoint changes every loop
        // even when the hub angle is stable; without this guard the motor chases the
        // noise and makes continuous audible buzzing/hunting noise.
        //
        // Bypass during SOTM (trackingRateRadPerSec ≠ 0): the lead angle shifts every
        // loop as the robot drives; suppressing those updates would leave the turret
        // lagging its SOTM setpoint by up to TURRET_SETPOINT_HYSTERESIS_DEG.
        // The 0.1 rad/s threshold ≈ 6 °/s — well below any meaningful robot motion.
        boolean robotStationary = Math.abs(trackingRateRadPerSec) < 0.1;
        if (isAligned()
                && robotStationary
                && Math.abs(clampedEncoderDeg - m_lastCommandedEncoderDeg)
                   < Turret.TURRET_SETPOINT_HYSTERESIS_DEG) {
            return;
        }
        m_lastCommandedEncoderDeg = clampedEncoderDeg;

        double motorRot = Units.degreesToRotations(clampedEncoderDeg) * Turret.TURRET_GEAR_RATIO;

        // Spring feedforward — compensates for the torsion spring so the PID does not
        // fight it.  The spring equilibrium (zero torque) is at TURRET_CABLE_HOME_ENCODER_DEG
        // (-135°), NOT at encoder 0.  At boot (encoder 0 = forward), the spring is already
        // pulled 135° from equilibrium and exerts significant torque.
        // FF = (currentEncoder − cableHome) × KF:
        //   encoder > cableHome → turret is CW of equilibrium → spring pulls turret CCW
        //   → through gears that CW torque loads motor in the CW direction
        //   → motor must resist with CCW output (positive Phoenix) → positive FF ✓
        double currentEncoderDeg = Units.rotationsToDegrees(
                m_turret.getPosition().getValueAsDouble() / Turret.TURRET_GEAR_RATIO);
        double springFF = (currentEncoderDeg - Turret.TURRET_CABLE_HOME_ENCODER_DEG)
                        * Turret.TURRET_SPRING_KF;

        // Full tracking feedforward: pre-load the voltage to hold the turret on the
        // moving hub target.  trackingRateRadPerSec = ω + vLateral/d encodes both
        // robot rotation AND turret-pivot lateral translation (see Javadoc above).
        double trackingVelRps = trackingRateRadPerSec / (2.0 * Math.PI) * Turret.TURRET_GEAR_RATIO;
        double trackingFF = Turret.TURRET_KV * trackingVelRps;

        SmartDashboard.putNumber("Turret/TrackingFFVolts", trackingFF);
        SmartDashboard.putNumber("Turret/TrackingVelRps",  trackingVelRps);

        m_turret.setControl(m_positionReq.withPosition(motorRot).withFeedForward(springFF + trackingFF));
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
     * Returns whether the turret is within the tight static tolerance.
     * Used to gate the PREPPING_TO_SHOOT → SHOOTING transition when stationary.
     *
     * @return {@code true} if the angular error is ≤ {@link Turret#TURRET_TOLERANCE_DEG}.
     */
    public boolean isAligned() {
        return getErrorDeg() <= Turret.TURRET_TOLERANCE_DEG;
    }

    /**
     * Returns whether the turret is within the wider <em>moving</em> tolerance.
     * Used to gate the feeder during continuous fire while the robot is driving
     * and the setpoint updates every loop.
     *
     * <p>Mirrors {@link frc.robot.subsystems.ShooterSubsystem#isFlywheelTracking()}.
     *
     * @return {@code true} if the angular error is ≤ {@link Turret#TURRET_MOVING_TOLERANCE_DEG}.
     */
    public boolean isTracking() {
        return getErrorDeg() <= Turret.TURRET_MOVING_TOLERANCE_DEG;
    }

    /**
     * Returns the absolute angular error between the turret's current measured
     * position and the last commanded target.
     *
     * <p><b>Cable-limited turret:</b> Because the turret cannot rotate through
     * 360° (it is limited by the cable wrap between
     * {@link Turret#TURRET_REVERSE_LIMIT_DEG} and
     * {@link Turret#TURRET_FORWARD_LIMIT_DEG}), the motor must always travel
     * the <em>linear</em> distance within that range.  The correct error is
     * simply {@code |current − target|} — no angular modulus wrapping.
     *
     * <p>Both {@link #getAngleDeg()} and {@code m_targetAngleDeg} are
     * guaranteed to lie within the same 360° cable-travel range (by encoder
     * soft limits and the {@link #setAngle} inputModulus clamp), so the
     * difference directly equals the physical travel distance.
     *
     * @return Error in degrees (always ≥ 0).
     */
    public double getErrorDeg() {
        return Math.abs(getAngleDeg() - m_targetAngleDeg);
    }

    /**
     * Computes the physical travel distance the turret would need to reach the
     * specified target angle from its current measured position, <b>without</b>
     * actually commanding the move.
     *
     * <p>Replicates the {@link #setAngle} input-modulus and clamp logic to
     * derive the final encoder-space target, then returns the absolute distance
     * from the current position.  Used by
     * {@link frc.robot.superstructure.Superstructure#commandTurretAngle} for
     * proactive wraparound detection — the Superstructure can decide whether
     * to enter WRAPAROUND <em>before</em> the motor is commanded.
     *
     * @param requestedAngleDeg Target turret angle in degrees (robot-relative,
     *                          same convention as {@link #setAngle}).
     * @return Physical travel distance in degrees (always ≥ 0).
     */
    public double getRequiredTravelDeg(double requestedAngleDeg) {
        double encoderAngleDeg = -requestedAngleDeg;
        encoderAngleDeg = MathUtil.inputModulus(
                encoderAngleDeg,
                Turret.TURRET_REVERSE_LIMIT_DEG,
                Turret.TURRET_FORWARD_LIMIT_DEG);
        double clampedEncoderDeg = Math.max(Turret.TURRET_REVERSE_LIMIT_DEG,
                                   Math.min(Turret.TURRET_FORWARD_LIMIT_DEG, encoderAngleDeg));
        double finalTargetDeg = -clampedEncoderDeg;
        return Math.abs(getAngleDeg() - finalTargetDeg);
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
        // Convert encoder angle → robot-relative angle.
        // Encoder 0 = forward; negate for gear inversion (motor CCW = turret CW).
        double encoderDeg = Units.rotationsToDegrees(
                m_turret.getPosition().getValueAsDouble() / Turret.TURRET_GEAR_RATIO);
        return -encoderDeg;
    }

    /** @return Last turret target in degrees. */
    public double getTargetAngleDeg() { return m_targetAngleDeg; }

    /**
     * Returns the turret's current angular velocity in degrees per second.
     * Positive = CCW (same sign convention as {@link #getAngleDeg()}).
     * Used by {@link frc.robot.superstructure.Superstructure} to predict the
     * turret's position at the moment the ball exits the barrel.
     */
    public double getVelocityDegPerSec() {
        double motorRotPerSec = m_turret.getVelocity().getValueAsDouble();
        // Same inversion as getAngleDeg(): motor CCW = turret CW → negate
        return -Units.rotationsToDegrees(motorRotPerSec / Turret.TURRET_GEAR_RATIO);
    }

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

    /**s
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
