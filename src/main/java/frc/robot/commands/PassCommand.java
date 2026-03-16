package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FieldLayout;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;
import frc.robot.util.ShooterKinematics;
import frc.robot.util.ShooterKinematics.ShooterSetpoint;

/**
 * Passes balls toward a specific field position where an alliance partner can
 * collect them.  The target is chosen based on which side of the hub the robot
 * is currently on (split at the field Y-centre = FIELD_WIDTH_M / 2):
 *
 * <ul>
 *   <li><b>Right side</b> (robot Y &lt; field centre): midpoint of the bottom wall
 *       and hub right face (Y), midpoint of the alliance wall and hub front face (X).</li>
 *   <li><b>Left side</b>  (robot Y &ge; field centre): midpoint of hub left face and
 *       top wall (Y), midpoint of the alliance wall and hub geometric centre (X).</li>
 * </ul>
 *
 * <p>The side is latched at command {@link #initialize()} so the target does not
 * jump if the robot crosses the centre line during the pass.
 *
 * <p>Uses the same Hybrid-TOF SOTM algorithm as {@link ShootCommand}: the turret
 * lead angle and RPM/hood setpoints are adjusted for the robot's current velocity
 * and feeder latency, so passes are accurate whether the robot is stationary or
 * moving.
 *
 * <p>Hold the button to pass; releasing returns to {@link RobotState#STOWED}.
 */
public class PassCommand extends Command {

    private final Superstructure          m_superstructure;
    private final CommandSwerveDrivetrain m_drivetrain;

    /** Pass target latched on initialize() — does not change mid-command. */
    private Translation2d m_passTarget;

    // EMA for chassis velocity (same filter as ShootCommand)
    private double  m_filtVx    = 0.0;
    private double  m_filtVy    = 0.0;
    private double  m_filtOmega = 0.0;
    private boolean m_velSeeded = false;

    public PassCommand(Superstructure superstructure,
                       CommandSwerveDrivetrain drivetrain) {
        m_superstructure = superstructure;
        m_drivetrain     = drivetrain;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        m_passTarget = selectPassTarget();
        m_velSeeded  = false;
        m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
    }

    @Override
    public void execute() {
        RobotState cur = m_superstructure.getState();
        if (cur != RobotState.PASSING_TO_ALLIANCE
                && cur != RobotState.EXHAUSTING) {
            m_superstructure.requestState(RobotState.PASSING_TO_ALLIANCE);
        }

        // ── Target vector in robot frame ─────────────────────────────────────
        Translation2d turretPos = getTurretPivotPosition();
        var robotPose = m_drivetrain.getState().Pose;
        Translation2d delta      = m_passTarget.minus(turretPos);
        Translation2d deltaRobot = delta.rotateBy(robotPose.getRotation().unaryMinus());
        double targetDist  = delta.getNorm();
        double alphaNowRad = Math.atan2(deltaRobot.getY(), deltaRobot.getX());

        // ── EMA velocity ──────────────────────────────────────────────────────
        ChassisSpeeds rawSpd = m_drivetrain.getState().Speeds;
        if (!m_velSeeded) {
            m_filtVx    = rawSpd.vxMetersPerSecond;
            m_filtVy    = rawSpd.vyMetersPerSecond;
            m_filtOmega = rawSpd.omegaRadiansPerSecond;
            m_velSeeded = true;
        } else {
            double a = Shooter.SOTM_VEL_ALPHA;
            m_filtVx    += a * (rawSpd.vxMetersPerSecond     - m_filtVx);
            m_filtVy    += a * (rawSpd.vyMetersPerSecond     - m_filtVy);
            m_filtOmega += a * (rawSpd.omegaRadiansPerSecond - m_filtOmega);
        }
        double vx    = m_filtVx;
        double vy    = m_filtVy;
        double omega = m_filtOmega;
        double chassisSpeedMps = Math.hypot(vx, vy);

        // Turret pivot velocity in robot frame (includes ω × offset cross-term)
        double vxT = vx - omega * Turret.TURRET_OFFSET_Y_M;
        double vyT = vy + omega * Turret.TURRET_OFFSET_X_M;

        // Hub vector (target vector) in robot frame
        double hubX = targetDist * Math.cos(alphaNowRad);
        double hubY = targetDist * Math.sin(alphaNowRad);

        // ── SOTM: Hybrid TOF with latency compensation ────────────────────────
        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS
                            && Math.abs(omega) < 0.3;

        double dFire;
        double alphaFireRad;
        ShooterSetpoint setpoint;

        if (isStationary) {
            dFire        = clampRange(targetDist);
            alphaFireRad = alphaNowRad;
            setpoint     = ShooterKinematics.calculatePass(dFire);
        } else {
            final double decay   = Shooter.SOTM_DRAG_DECAY_FACTOR;
            final double latency = Shooter.SOTM_LATENCY_S;

            double seedDist = clampRange(targetDist);
            double tof = ShooterKinematics.getFlightTimeSeconds(
                    seedDist, ShooterKinematics.calculatePass(seedDist));

            double fireX = hubX, fireY = hubY;
            dFire        = seedDist;
            alphaFireRad = alphaNowRad;
            setpoint     = ShooterKinematics.calculatePass(dFire);

            for (int i = 0; i < 2; i++) {
                fireX        = hubX - vxT * (tof + latency) * decay;
                fireY        = hubY - vyT * (tof + latency) * decay;
                dFire        = clampRange(Math.hypot(fireX, fireY));
                alphaFireRad = Math.atan2(fireY, fireX);
                setpoint     = ShooterKinematics.calculatePass(dFire);
                tof          = ShooterKinematics.getFlightTimeSeconds(dFire, setpoint);
            }
            // Final position at converged tof
            fireX        = hubX - vxT * (tof + latency) * decay;
            fireY        = hubY - vyT * (tof + latency) * decay;
            dFire        = clampRange(Math.hypot(fireX, fireY));
            alphaFireRad = Math.atan2(fireY, fireX);
            setpoint     = ShooterKinematics.calculatePass(dFire);
        }

        // ── Apply setpoints ───────────────────────────────────────────────────
        m_superstructure.applyShooterSetpoint(setpoint);
        double vLateral = -vxT * Math.sin(alphaFireRad) + vyT * Math.cos(alphaFireRad);
        m_superstructure.commandTurretAngle(Math.toDegrees(alphaFireRad), vLateral, dFire);

        SmartDashboard.putNumber("Pass/DistanceM",    targetDist);
        SmartDashboard.putNumber("Pass/DFireM",       dFire);
        SmartDashboard.putNumber("Pass/AlphaFireDeg", Math.toDegrees(alphaFireRad));
        SmartDashboard.putNumber("Pass/TargetX",      m_passTarget.getX());
        SmartDashboard.putNumber("Pass/TargetY",      m_passTarget.getY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_superstructure.requestState(RobotState.STOWED);
    }

    // =========================================================================
    // Private Helpers
    // =========================================================================

    /**
     * Picks the pass target based on the robot's current Y position and alliance.
     * Right side = Y < FIELD_WIDTH_M/2 (bottom/right wall side).
     * Left side  = Y >= FIELD_WIDTH_M/2 (top/left wall side).
     */
    private Translation2d selectPassTarget() {
        boolean isRed = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        double robotY = m_drivetrain.getState().Pose.getTranslation().getY();
        boolean isRightSide = robotY < FieldLayout.FIELD_WIDTH_M / 2.0;

        if (isRed) {
            return isRightSide ? FieldLayout.RED_PASS_TARGET_RIGHT : FieldLayout.RED_PASS_TARGET_LEFT;
        } else {
            return isRightSide ? FieldLayout.BLUE_PASS_TARGET_RIGHT : FieldLayout.BLUE_PASS_TARGET_LEFT;
        }
    }

    /** Returns the turret pivot position in field coordinates. */
    private Translation2d getTurretPivotPosition() {
        var robotPose = m_drivetrain.getState().Pose;
        return robotPose.getTranslation().plus(
                new Translation2d(Turret.TURRET_OFFSET_X_M, Turret.TURRET_OFFSET_Y_M)
                        .rotateBy(robotPose.getRotation()));
    }

    /** Clamps distance to the shootable range. */
    private double clampRange(double dist) {
        return Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
               Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, dist));
    }
}
