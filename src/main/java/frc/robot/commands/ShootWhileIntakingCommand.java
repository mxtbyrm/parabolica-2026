package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
 * Simultaneous intake-and-shoot command ("spray and pray").
 *
 * <p>Deploys the intake and runs the spindexer continuously while the flywheel
 * tracks the HUB and fires as fast as new balls are available.  The Superstructure
 * handles feeder gating — the feeder only engages once
 * {@link frc.robot.subsystems.ShooterSubsystem#isFlywheelAtSpeed()} is true,
 * so no balls are wasted during the initial spin-up.
 *
 * <h2>Moving-While-Shooting Compensation</h2>
 * <p>Identical math to {@link ShootCommand}: velocity decomposition into radial
 * and lateral components, 2-pass {@code d_eff} convergence, flywheel slew-rate
 * limiting, and turret lead angle.  Unlike {@link ShootCommand}, there is no
 * readiness gate before the Superstructure begins firing — the state machine
 * manages that internally.
 *
 * <h2>Lifecycle</h2>
 * <ul>
 *   <li><b>initialize</b> — seed slew state; request {@link RobotState#SHOOT_WHILE_INTAKING}
 *       (or {@link RobotState#PASSING_TO_ALLIANCE} if the HUB is already inactive).</li>
 *   <li><b>execute (active period)</b> — hub-tracking setpoints + re-request
 *       {@link RobotState#SHOOT_WHILE_INTAKING} each loop (recovers from any
 *       INACTIVE→PASSING→PREPPING transition automatically).</li>
 *   <li><b>execute (inactive period)</b> — request {@link RobotState#PASSING_TO_ALLIANCE}
 *       and aim turret at the alliance wall dynamically; Superstructure handles
 *       fixed flywheel/hood/feeder setpoints.</li>
 *   <li><b>isFinished</b> — always {@code false}; runs until the operator releases
 *       the button or the command is interrupted.</li>
 *   <li><b>end (interrupted)</b>  — {@link RobotState#STOWED}.</li>
 *   <li><b>end (natural, button released)</b> — {@link RobotState#PREPPING_TO_SHOOT}
 *       (flywheel stays warm for a follow-up shot).</li>
 * </ul>
 *
 * <p>{@code addRequirements}: superstructure, vision — NOT drivetrain (same as
 * {@link ShootCommand}, so driving is still possible while this command runs).
 */
public class ShootWhileIntakingCommand extends Command {

    // =========================================================================
    // Dependencies
    // =========================================================================

    private final Superstructure          m_superstructure;
    private final VisionSubsystem         m_vision;
    private final CommandSwerveDrivetrain m_drivetrain;

    // =========================================================================
    // Command State
    // =========================================================================

    private double          m_lastDistanceM = 4.0; // safe starting assumption
    private ShooterSetpoint m_lastSetpoint;

    /**
     * Slew-rate-limited flywheel RPM — same semantics as in {@link ShootCommand}.
     * May only decrease at {@link Shooter#FLYWHEEL_SLEW_RATE_DOWN_RPM_PER_S};
     * spin-up is unlimited.
     */
    private double m_smoothedFlywheelRPM = 0.0;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Constructs a ShootWhileIntakingCommand.
     *
     * @param superstructure The superstructure state machine.
     * @param vision         The vision subsystem (distance, tx, tag selection).
     * @param drivetrain     The swerve drivetrain (robot-relative chassis speeds
     *                       used for moving-while-shooting compensation; not
     *                       required as a subsystem so driving remains available).
     */
    public ShootWhileIntakingCommand(Superstructure superstructure,
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
        m_lastSetpoint = ShooterKinematics.calculate(m_lastDistanceM);
        // Seed slew state so the flywheel immediately targets the correct RPM.
        m_smoothedFlywheelRPM = m_lastSetpoint.flywheelRPM();
        m_superstructure.requestState(RobotState.SHOOT_WHILE_INTAKING);
    }

    @Override
    public void execute() {
        // Inactive period: aim turret at alliance wall dynamically, let
        // Superstructure handle fixed flywheel/hood/feeder setpoints.
        if (HubStateMonitor.getHubState() == HubState.INACTIVE) {
            m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
            commandTurretToAllianceWall();
            return;
        }

        // =====================================================================
        // PHASE 1 — COMPUTE ALL SETPOINTS
        // =====================================================================

        // --- Velocity decomposition relative to turret axis ------------------
        ChassisSpeeds robotSpeeds = m_drivetrain.getState().Speeds;
        double turretRad = Math.toRadians(m_superstructure.getTurretAngleDeg());

        //   v_radial  > 0  →  robot moving toward hub along turret axis
        //   v_lateral > 0  →  robot moving left (CCW) relative to turret axis
        double vRadial  =  robotSpeeds.vxMetersPerSecond * Math.cos(turretRad)
                         + robotSpeeds.vyMetersPerSecond * Math.sin(turretRad);
        double vLateral = -robotSpeeds.vxMetersPerSecond * Math.sin(turretRad)
                         + robotSpeeds.vyMetersPerSecond * Math.cos(turretRad);

        // Robot spin contribution: turret pivot sweeps tangentially at v = omega * R.
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
        double rawRPM     = m_lastSetpoint.flywheelRPM();
        double maxDropRPM = Shooter.FLYWHEEL_SLEW_RATE_DOWN_RPM_PER_S * 0.020; // per 20 ms loop
        if (rawRPM < m_smoothedFlywheelRPM - maxDropRPM) {
            m_smoothedFlywheelRPM -= maxDropRPM;
        } else {
            m_smoothedFlywheelRPM = rawRPM;
        }

        // --- Turret: vision correction + lateral lead angle ------------------
        double tFlight = tFlight2;
        double leadAngleDeg = (m_lastDistanceM > 0 && tFlight > 0)
                ? Math.toDegrees(Math.atan2(-vLateral * tFlight, m_lastDistanceM))
                : 0.0;

        // Vision tx takes priority; odometry used as fallback when vision is
        // disabled or tag not visible.
        double[] turretTargetDeg = {Double.NaN};
        m_vision.getTargetTxDeg().ifPresent(tx -> {
            turretTargetDeg[0] = m_superstructure.getTurretAngleDeg() - tx + leadAngleDeg;
        });
        if (Double.isNaN(turretTargetDeg[0])) {
            m_vision.getHubRobotRelativeAngleDeg().ifPresent(robotAngleDeg -> {
                turretTargetDeg[0] = robotAngleDeg + leadAngleDeg;
            });
        }

        // =====================================================================
        // PHASE 2 — SEND ALL SETPOINTS (flywheel, hood, turret) AT ONCE
        // =====================================================================

        m_superstructure.applyShooterSetpoint(
                new ShooterSetpoint(m_smoothedFlywheelRPM, m_lastSetpoint.hoodAngleDeg()));

        if (!Double.isNaN(turretTargetDeg[0])) {
            m_superstructure.commandTurretAngle(turretTargetDeg[0]);
        }

        // Re-request SHOOT_WHILE_INTAKING in case we are returning from an inactive
        // period (Superstructure auto-transitioned to PREPPING_TO_SHOOT when the
        // active period resumed).  requestState() is idempotent — no-op if we are
        // already in SHOOT_WHILE_INTAKING.  The m_feedingStarted latch in Superstructure
        // ensures the feeder waits for flywheel speed before engaging on re-entry.
        m_superstructure.requestState(RobotState.SHOOT_WHILE_INTAKING);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until the operator releases the button (or interrupted).
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

    /**
     * Points the turret toward the robot's own alliance wall based on the robot's
     * current field-relative heading and the active alliance reported by the DS.
     * See {@link frc.robot.commands.ShootCommand#commandTurretToAllianceWall()} for
     * the coordinate-system derivation.
     */
    private void commandTurretToAllianceWall() {
        double robotHeadingDeg = m_drivetrain.getState().Pose.getRotation().getDegrees();
        double wallFieldAngleDeg =
                (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                        == DriverStation.Alliance.Red)
                ? 0.0 : 180.0;
        m_superstructure.commandTurretAngle(wallFieldAngleDeg - robotHeadingDeg);
    }
}
