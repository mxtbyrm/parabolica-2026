package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.Shooter;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;
import frc.robot.util.HubStateMonitor;
import frc.robot.util.HubStateMonitor.HubState;
import frc.robot.util.ShooterKinematics;
import frc.robot.util.ShooterKinematics.ShooterSetpoint;

/**
 * Full shoot-sequence command.  Coordinates vision, kinematics (including
 * moving-while-shooting compensation for both radial and lateral robot motion),
 * and the Superstructure state machine to score FUEL in the HUB.
 *
 * <h2>Execution Sequence</h2>
 * <ol>
 *   <li><b>Initialize</b> — Verify the HUB is scoring-eligible.  If not, the
 *       command finishes immediately.  Otherwise, request
 *       {@link RobotState#PREPPING_TO_SHOOT}.</li>
 *   <li><b>Execute (each loop)</b>
 *     <ol type="a">
 *       <li>Abort to {@link RobotState#STOWED} if the HUB becomes inactive.</li>
 *       <li>Decompose robot velocity into <em>radial</em> (along turret axis) and
 *           <em>lateral</em> (perpendicular to turret axis) components.</li>
 *       <li>Compute an effective distance that accounts for the robot closing or
 *           opening the gap to the HUB during ball flight.</li>
 *       <li>Recalculate the {@link ShooterSetpoint} — both flywheel RPM <em>and</em>
 *           hood angle — at the effective distance.</li>
 *       <li>Add a turret lead angle (on top of the vision correction) to compensate
 *           for lateral robot motion during ball flight.</li>
 *       <li>Request {@link RobotState#SHOOTING} once all readiness conditions are
 *           met and the distance is in range.</li>
 *     </ol>
 *   </li>
 *   <li><b>End (natural)</b> — Hold {@link RobotState#PREPPING_TO_SHOOT} so the
 *       flywheel stays warm for a rapid follow-up shot.</li>
 *   <li><b>End (interrupted)</b> — Return to {@link RobotState#STOWED}.</li>
 * </ol>
 *
 * <h2>Moving-While-Shooting Compensation</h2>
 *
 * <p>The robot's velocity (from robot-relative chassis speeds) is decomposed into
 * two components along and perpendicular to the current turret axis.  This is correct
 * even when the turret is rotated off-center.
 *
 * <h3>Radial velocity — affects flywheel RPM and hood angle</h3>
 * <pre>
 *   d_eff = d_vision − v_radial × t_flight
 * </pre>
 * <p>If the robot drives <em>toward</em> the HUB ({@code v_radial > 0}), the ball
 * arrives at a shorter distance ({@code d_eff < d_vision}), requiring less flywheel
 * energy and a steeper hood angle.  The reverse applies when driving away.
 * Both flywheel RPM and hood angle are re-computed each loop at {@code d_eff}.
 *
 * <h3>Lateral velocity — affects turret lead angle</h3>
 * <pre>
 *   v_lateral_total = v_lateral_translation + ω × R_turret
 *   δθ = arctan( −v_lateral_total × t_flight / d_vision )
 * </pre>
 * <p>If the robot moves left relative to the turret axis ({@code v_lateral > 0}),
 * the HUB will have drifted right by the time the ball arrives.  The turret leads
 * right ({@code δθ < 0}, CW) to intercept.  Robot spin ({@code ω}) adds an
 * additional tangential velocity at the turret pivot ({@code ω × R}) that
 * displaces the ball in the lateral direction and must also be compensated.
 */
public class ShootCommand extends Command {

    // =========================================================================
    // Dependencies
    // =========================================================================

    private final Superstructure          m_superstructure;
    private final VisionSubsystem         m_vision;
    private final CommandSwerveDrivetrain m_drivetrain;

    // =========================================================================
    // Command State
    // =========================================================================

    private boolean         m_hubWasEligibleOnInit;
    private double          m_lastDistanceM = 4.0; // safe starting assumption
    private ShooterSetpoint m_lastSetpoint;

    /**
     * Slew-rate-limited flywheel RPM sent to the superstructure.
     *
     * <p>Tracks {@code m_lastSetpoint.flywheelRPM()} but may only <em>decrease</em>
     * at {@link Shooter#FLYWHEEL_SLEW_RATE_DOWN_RPM_PER_S}.  This prevents the
     * dEff correction from instantly slashing the setpoint when the robot approaches
     * the HUB quickly, keeping the flywheel within {@code isFlywheelAtSpeed()} tolerance
     * so shoot-while-moving can actually fire.  Spin-up is unlimited.
     */
    private double m_smoothedFlywheelRPM = 0.0;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Constructs a ShootCommand.
     *
     * @param superstructure The superstructure state machine.
     * @param vision         The vision subsystem (distance, tx, tag selection).
     * @param drivetrain     The swerve drivetrain (robot-relative chassis speeds
     *                       used for moving-while-shooting compensation).
     */
    public ShootCommand(Superstructure superstructure,
                        VisionSubsystem vision,
                        CommandSwerveDrivetrain drivetrain) {
        m_superstructure = superstructure;
        m_vision         = vision;
        m_drivetrain     = drivetrain;
        addRequirements(superstructure, vision);
    }

    // =========================================================================
    // Command Lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        m_hubWasEligibleOnInit = HubStateMonitor.isSafeToBeginShot();

        if (!m_hubWasEligibleOnInit) {
            return; // Will finish immediately — no mechanism movement.
        }

        m_lastSetpoint = ShooterKinematics.calculate(m_lastDistanceM);
        // Seed the slew state so the flywheel targets the correct RPM immediately;
        // avoids an artificially low first-loop setpoint that would need to ramp up.
        m_smoothedFlywheelRPM = m_lastSetpoint.flywheelRPM();
        m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
    }

    @Override
    public void execute() {
        if (!m_hubWasEligibleOnInit) {
            return;
        }

        // --- Guard: abort if HUB deactivates mid-sequence -------------------
        if (HubStateMonitor.getHubState() == HubState.INACTIVE) {
            m_superstructure.requestState(RobotState.STOWED);
            return;
        }

        // =====================================================================
        // PHASE 1 — COMPUTE ALL SETPOINTS
        // =====================================================================

        // --- Velocity decomposition relative to turret axis ------------------
        // getState().Speeds returns robot-relative chassis speeds (vx = forward,
        // vy = left, CCW-positive).  Decomposing by turret angle keeps the math
        // correct even when the turret is rotated off center-forward.
        ChassisSpeeds robotSpeeds = m_drivetrain.getState().Speeds;
        double turretRad = Math.toRadians(m_superstructure.getTurretAngleDeg());

        //   v_radial  > 0  →  robot moving toward hub along turret axis
        //   v_lateral > 0  →  robot moving left (CCW) relative to turret axis
        double vRadial  =  robotSpeeds.vxMetersPerSecond * Math.cos(turretRad)
                         + robotSpeeds.vyMetersPerSecond * Math.sin(turretRad);
        double vLateral = -robotSpeeds.vxMetersPerSecond * Math.sin(turretRad)
                         + robotSpeeds.vyMetersPerSecond * Math.cos(turretRad);

        // Robot spin contribution: the turret pivot (at radius R from the robot's
        // rotation center) sweeps tangentially at v = omega * R.  CCW robot spin
        // (omega > 0) moves the barrel tip left in the turret frame → positive lateral.
        vLateral += robotSpeeds.omegaRadiansPerSecond * Turret.TURRET_RADIUS_FROM_CENTER_M;

        // --- Update raw vision distance (only when a tag is visible) ---------
        m_vision.getDistanceToHubMeters().ifPresent(dist -> {
            if (dist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                    && dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                m_lastDistanceM = dist;
            }
        });

        // --- 2-pass d_eff convergence ----------------------------------------
        // Pass 1: estimate t_flight using the previous loop's setpoint.
        double tFlight1 = (m_lastSetpoint != null)
                ? ShooterKinematics.getFlightTimeSeconds(m_lastDistanceM, m_lastSetpoint)
                : 0.0;

        double dEff1 = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                       Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M,
                                m_lastDistanceM - vRadial * tFlight1));

        // Pass 2: refine with a more accurate t_flight at the pass-1 setpoint.
        ShooterSetpoint pass1Setpoint = ShooterKinematics.calculate(dEff1);
        double tFlight2 = ShooterKinematics.getFlightTimeSeconds(dEff1, pass1Setpoint);

        double dEff = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                      Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M,
                               m_lastDistanceM - vRadial * tFlight2));

        // --- Final setpoint at converged effective distance ------------------
        m_lastSetpoint = ShooterKinematics.calculate(dEff);

        // --- Flywheel slew-rate limiting -------------------------------------
        // Commanded RPM may only DROP at FLYWHEEL_SLEW_RATE_DOWN_RPM_PER_S so
        // isFlywheelAtSpeed() stays true when approaching the HUB quickly.
        // Spin-up is unlimited so the flywheel reaches speed immediately.
        double rawRPM     = m_lastSetpoint.flywheelRPM();
        double maxDropRPM = Shooter.FLYWHEEL_SLEW_RATE_DOWN_RPM_PER_S * 0.020; // per 20 ms loop
        if (rawRPM < m_smoothedFlywheelRPM - maxDropRPM) {
            m_smoothedFlywheelRPM -= maxDropRPM;
        } else {
            m_smoothedFlywheelRPM = rawRPM;
        }

        // --- Turret: vision correction + lateral lead angle ------------------
        // Positive vLateral (robot moving left) → hub drifts right at ball arrival
        // → turret leads right (negative / CW) → atan2(-vLateral·t, d) < 0  ✓
        double tFlight = tFlight2;
        double leadAngleDeg = (m_lastDistanceM > 0 && tFlight > 0)
                ? Math.toDegrees(Math.atan2(-vLateral * tFlight, m_lastDistanceM))
                : 0.0;

        // Capture the full turret target; present only when a hub tag is visible.
        // tx > 0 → tag is right of camera crosshair → rotate turret CW (negative).
        double[] turretTargetDeg = {Double.NaN};
        m_vision.getTargetTxDeg().ifPresent(tx -> {
            turretTargetDeg[0] = m_superstructure.getTurretAngleDeg() - tx + leadAngleDeg;
        });

        // =====================================================================
        // PHASE 2 — SEND ALL SETPOINTS (flywheel, hood, turret) AT ONCE
        // =====================================================================

        m_superstructure.applyShooterSetpoint(
                new ShooterSetpoint(m_smoothedFlywheelRPM, m_lastSetpoint.hoodAngleDeg()));

        if (!Double.isNaN(turretTargetDeg[0])) {
            m_superstructure.commandTurretAngle(turretTargetDeg[0]);
        }

        // --- Transition to SHOOTING when all conditions are satisfied --------
        if (m_superstructure.getState() == RobotState.PREPPING_TO_SHOOT
                && m_superstructure.isReadyToShoot()
                && isDistanceInRange()
                && HubStateMonitor.isSafeToBeginShot()) {
            m_superstructure.requestState(RobotState.SHOOTING);
        }
    }

    @Override
    public boolean isFinished() {
        if (!m_hubWasEligibleOnInit) {
            return true; // HUB was inactive on init — end without doing anything.
        }
        return HubStateMonitor.getHubState() == HubState.INACTIVE;
    }

    /**
     * On natural completion: hold {@link RobotState#PREPPING_TO_SHOOT} so the flywheel
     * stays at speed for a faster follow-up shot.
     * On interruption: return to {@link RobotState#STOWED}.
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_superstructure.requestState(RobotState.STOWED);
        } else {
            m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
        }
    }

    // =========================================================================
    // Private Helpers
    // =========================================================================

    /** Returns whether the last known vision distance is within the configured shooting range. */
    private boolean isDistanceInRange() {
        return m_lastDistanceM >= SuperstructureConstants.MIN_SHOOT_RANGE_M
            && m_lastDistanceM <= SuperstructureConstants.MAX_SHOOT_RANGE_M;
    }
}
