package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.Shooter;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

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
    private final PhotonVisionSubsystem  m_photonVision;

    // =========================================================================
    // Command State
    // =========================================================================

    /** Last in-range distance used for setpoint computation (clamped, never out-of-range). */
    private double  m_lastDistanceM    = 4.0;

    /** Most recent raw distance from vision this loop (un-clamped; may be out of range). */
    private double  m_rawDistanceM     = 4.0;
    /** True when vision provided a distance measurement during the current execute() loop. */
    private boolean m_rawDistanceValid = false;

    /**
     * Consecutive loops where physicsOK has been false while in SHOOTING.
     * Drop-back to PREPPING_TO_SHOOT only fires after {@link #PHYSICS_NOT_OK_DROP_LOOPS}
     * consecutive failures, preventing single-loop distance glitches from cutting
     * the feeder during rapid robot movement.
     */
    private int m_physicsNotOkCount = 0;
    private static final int PHYSICS_NOT_OK_DROP_LOOPS = 5; // ~100 ms

    // EMA state for chassis velocity — seeded at initialize(), updated each loop.
    // y_n = α·x_n + (1−α)·y_{n−1}  (α = SOTM_VEL_ALPHA, τ ≈ 39 ms at 20 ms loop)
    private double  m_filtVx    = 0.0;
    private double  m_filtVy    = 0.0;
    private double  m_filtOmega = 0.0;

    // EMA state for turret-pivot acceleration via finite difference of turret-pivot velocity.
    // Source is turret-pivot (not robot center) because the ball exits from the pivot.
    // This correctly captures centripetal terms (ω×offset, α×offset) automatically.
    // (α = SOTM_ACCEL_ALPHA, τ ≈ 72 ms — heavier smoothing needed for derivative)
    private double  m_filtAxT   = 0.0;
    private double  m_filtAyT   = 0.0;
    // Previous turret-pivot velocity — used for the finite-difference numerator.
    private double  m_prevVxT   = 0.0;
    private double  m_prevVyT   = 0.0;

    private boolean m_velSeeded = false;


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
     * @param photonVision   The PhotonVision subsystem (raw-pose hub angle).
     */
    public ShootCommand(Superstructure superstructure,
                        VisionSubsystem vision,
                        CommandSwerveDrivetrain drivetrain,
                        PhotonVisionSubsystem photonVision) {
        m_superstructure = superstructure;
        m_vision         = vision;
        m_drivetrain     = drivetrain;
        m_photonVision   = photonVision;
        addRequirements(superstructure, vision);
    }

    // =========================================================================
    // Command Lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        m_physicsNotOkCount = 0;
        // Force re-seed on first execute() loop so EMA starts from current state,
        // not stale or zeroed values from a previous command invocation.
        m_velSeeded = false;
        m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
    }

    @Override
    public void execute() {
        // --- State recovery --------------------------------------------------
        // Two recovery cases while this command is still running:
        // 1. STOWED: TrenchTraversalManager overrode to TRAVERSING_TRENCH then
        //    released — without recovery the command deadlocks (handleStowed()
        //    stops actuators every loop; execute() never transitions out).
        // 2. TRAVERSING_TRENCH: robot was in the approach zone when the operator
        //    pressed shoot — recover to PREPPING so scoring can begin.
        RobotState currentState = m_superstructure.getState();
        if (currentState == RobotState.STOWED
                || currentState == RobotState.TRAVERSING_TRENCH) {
            m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
        }

        // =====================================================================
        // PHASE 1 — COMPUTE ALL SETPOINTS
        // =====================================================================

        // --- Update hub distance -------------------------------------------------
        // Priority: Odometry (fused pose estimator) → PhotonVision raw PnP fallback.
        // Consistent with angle priority (see below) — both use odometry primary.
        // Odometry is preferred because the fused pose already incorporates all PV
        // corrections via the Kalman filter.  PhotonVision raw distance is a fallback
        // for pre-match / simulation (unknown alliance).
        //
        // m_rawDistanceM  = actual measurement this loop (may be out of range).
        // m_lastDistanceM = last in-range measurement (used for setpoint computation).
        //
        // Keeping both is critical: isDistanceInRange() MUST use the raw value so
        // the gate correctly blocks shots when the robot drifts outside the range.
        // Using only m_lastDistanceM (which never updates when out-of-range) would
        // make isDistanceInRange() always return true after first lock.
        m_rawDistanceValid = false;
        m_vision.getFusedHubDistanceMeters().ifPresentOrElse(
            dist -> {
                m_rawDistanceM     = dist;
                m_rawDistanceValid = true;
                if (dist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                        && dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                    m_lastDistanceM = dist;
                }
            },
            () -> m_photonVision.getHubDistanceMeters().ifPresent(dist -> {
                m_rawDistanceM     = dist;
                m_rawDistanceValid = true;
                if (dist >= SuperstructureConstants.MIN_SHOOT_RANGE_M
                        && dist <= SuperstructureConstants.MAX_SHOOT_RANGE_M) {
                    m_lastDistanceM = dist;
                }
            })
        );


        // ── EMA State Estimation ─────────────────────────────────────────────────
        // Velocity: fast EMA (SOTM_VEL_ALPHA ≈ 0.40, τ ≈ 39 ms)
        // Acceleration: EMA on finite-difference of filtered velocity (SOTM_ACCEL_ALPHA ≈ 0.25, τ ≈ 72 ms)
        // Jerk is intentionally omitted: at T=FEEDER_TRANSIT_SECONDS (0.08s),
        //   jerk contribution to velocity = ½·j·T² ≈ 0.03 m/s → below sensor noise floor.
        ChassisSpeeds rawSpd = m_drivetrain.getState().Speeds;
        if (!m_velSeeded) {
            // First loop: seed EMA from current measurement so first-loop corrections
            // are accurate (cold-start at 0 would produce near-zero SOTM for 3–4 loops).
            m_filtVx    = rawSpd.vxMetersPerSecond;
            m_filtVy    = rawSpd.vyMetersPerSecond;
            m_filtOmega = rawSpd.omegaRadiansPerSecond;
            // Seed pivot velocity so first-loop acceleration finite-diff is 0.
            m_prevVxT = m_filtVx - m_filtOmega * Turret.TURRET_OFFSET_Y_M;
            m_prevVyT = m_filtVy + m_filtOmega * Turret.TURRET_OFFSET_X_M;
            m_filtAxT = 0.0;
            m_filtAyT = 0.0;
            m_velSeeded = true;
        } else {
            double aV = Shooter.SOTM_VEL_ALPHA;
            double aA = Shooter.SOTM_ACCEL_ALPHA;
            m_filtVx    += aV * (rawSpd.vxMetersPerSecond     - m_filtVx);
            m_filtVy    += aV * (rawSpd.vyMetersPerSecond     - m_filtVy);
            m_filtOmega += aV * (rawSpd.omegaRadiansPerSecond - m_filtOmega);
            // Acceleration from turret-pivot velocity finite difference.
            // Source is pivot (not robot center) because the ball exits from the pivot.
            // This automatically includes centripetal/angular terms (ω²·offset, α·offset).
            double vxT = m_filtVx - m_filtOmega * Turret.TURRET_OFFSET_Y_M;
            double vyT = m_filtVy + m_filtOmega * Turret.TURRET_OFFSET_X_M;
            m_filtAxT += aA * ((vxT - m_prevVxT) / 0.02 - m_filtAxT);
            m_filtAyT += aA * ((vyT - m_prevVyT) / 0.02 - m_filtAyT);
            m_prevVxT = vxT;
            m_prevVyT = vyT;
        }
        double vx    = m_filtVx;
        double vy    = m_filtVy;
        double omega = m_filtOmega;

        double chassisSpeedMps = Math.hypot(vx, vy);

        // ── Hub angle (vision) ────────────────────────────────────────────────
        // Priority: Odometry (fused pose estimator) → PhotonVision raw PnP fallback.
        // Consistent with distance priority — same fused pose source for both angle
        // and distance yields a coherent hub vector (hubX, hubY).  PhotonVision raw
        // PnP lags 1-2 loops under rapid repositioning and has oblique-angle PnP
        // errors; the Kalman-fused odometry angle is preferred.  PhotonVision is the
        // fallback for pre-match / simulation when the alliance (and hub position) is
        // unknown to odometry.  Both sources compute the angle from the turret pivot.
        double[] hubBaseAngleDeg = {Double.NaN};
        m_vision.getHubRobotRelativeAngleDeg().ifPresent(a -> hubBaseAngleDeg[0] = a);
        if (Double.isNaN(hubBaseAngleDeg[0])) {
            m_photonVision.getHubAngleDeg().ifPresent(a -> hubBaseAngleDeg[0] = a);
        }

        // ── Hub position vector in robot frame ────────────────────────────────
        // alphaNowRad: current robot-relative hub direction.  Falls back to the
        // current turret angle when vision is lost — odometry always has a value.
        // hubX/hubY are the hub's coordinates from the turret pivot, in robot frame.
        // Both the physics block (T ahead) and the turret command block (Tp ahead)
        // share these as their starting point.
        double alphaNowRad = Double.isNaN(hubBaseAngleDeg[0])
                ? Math.toRadians(m_superstructure.getTurretAngleDeg())
                : Math.toRadians(hubBaseAngleDeg[0]);
        double hubX = m_lastDistanceM * Math.cos(alphaNowRad);
        double hubY = m_lastDistanceM * Math.sin(alphaNowRad);

        // ── Turret-pivot velocity (robot frame) ───────────────────────────────
        // Ball exits from the pivot, not the robot center.
        // ω × offset cross-term: rotation moves the pivot even when vRobot = 0.
        double vxT = vx - omega * Turret.TURRET_OFFSET_Y_M;
        double vyT = vy + omega * Turret.TURRET_OFFSET_X_M;

        // Current lateral velocity at α_now — for turret tracking feedforward.
        // This is the instantaneous pivot lateral speed perpendicular to the
        // hub axis; it drives the motor's continuous tracking rate even between
        // shot cycles.
        double vLateralFF = -vxT * Math.sin(alphaNowRad) + vyT * Math.cos(alphaNowRad);

        // ── Predictive SOTM ───────────────────────────────────────────────────
        // T = FEEDER_TRANSIT_SECONDS: time from feeder-on to ball exit.
        //
        // COMPLETE VECTOR APPROACH — every kinematic term included:
        //
        //  1. Hub-from-pivot vector at fire time (2D, robot-frame approx):
        //       r_fire = (hubX − vxT·T − ½·axT·T²,
        //                 hubY − vyT·T − ½·ayT·T²)
        //     A single atan2/sqrt gives d_fire and α_fire exactly, capturing BOTH
        //     the rotational drift (−ω·T) AND the translational shift
        //     (Δα ≈ −vLateral·T/d, up to ~5° at 3 m/s lateral / 3 m distance).
        //     The previous hub − ω·T formula missed this translational term entirely.
        //
        //  2. Pivot velocity at fire time decomposed relative to α_fire.
        //     Decomposing at α_now (the old approach) is wrong when the hub
        //     direction has shifted, because the basis vectors are misaligned.
        //
        //  3. Required muzzle horizontal speed:
        //       v_h_needed = sqrt(vRcomp² + vLateralFire²)
        //     where vRcomp = d_fire/tFlight − vRadialFire.
        //     The sqrt term accounts for the extra muzzle energy needed to cancel
        //     lateral drift during flight (~8% RPM increase at 3 m/s lateral).
        //     Previously only the radial term was used — every lateral shot was short.
        //
        //  4. d_eff found by binary-searching the kinematics table for the
        //     distance that produces v_h = v_h_needed.  Two-iteration convergence
        //     resolves the tFlight/v_h circular dependency.
        //
        //  5. Lead angle = atan2(−vLateral, vRcomp) — exact for all quadrants.
        //     asin(−vLateral/v_h) diverges when vLateral approaches v_h and gives
        //     the wrong sign when vRcomp < 0 (fast retreat).
        final double T = Shooter.FEEDER_TRANSIT_SECONDS;

        double dEff;
        double leadAngleDeg;
        ShooterSetpoint setpoint;
        double vRadialDbg      = 0.0;
        double vLateralFireDbg = 0.0;

        boolean isWrapping   = m_superstructure.getState() == RobotState.WRAPAROUND;
        boolean isStationary = chassisSpeedMps < Shooter.SOTM_SPEED_DEADBAND_MPS
                            && Math.abs(omega) < 0.3;

        if (isWrapping || isStationary) {
            dEff         = m_lastDistanceM;
            leadAngleDeg = 0.0;
            setpoint     = ShooterKinematics.calculate(dEff);
        } else {
            // ── Step 1: Hub vector at fire time ───────────────────────────────
            // Pivot has moved by (vxT·T + ½·axT·T², vyT·T + ½·ayT·T²).
            // Subtracting from the current hub position gives the hub-from-pivot
            // vector at fire time.  Robot frame is non-inertial (rotates at ω),
            // but the positional error is ½|v|T²ω ≈ 3 cm at max field speeds —
            // causing < 1° turret error at d > 2 m.  Field-frame integration
            // would add significant complexity for no practical gain here.
            double fireX    = hubX - vxT * T - 0.5 * m_filtAxT * T * T;
            double fireY    = hubY - vyT * T - 0.5 * m_filtAyT * T * T;
            double dFire    = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                              Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M,
                                       Math.sqrt(fireX * fireX + fireY * fireY)));
            double alphaFireRad = Math.atan2(fireY, fireX);

            // ── Step 2: Pivot velocity at fire time, decomposed at α_fire ────
            double vxTFire      = vxT + m_filtAxT * T;
            double vyTFire      = vyT + m_filtAyT * T;
            double vRadialFire  =  vxTFire * Math.cos(alphaFireRad) + vyTFire * Math.sin(alphaFireRad);
            double vLateralFire = -vxTFire * Math.sin(alphaFireRad) + vyTFire * Math.cos(alphaFireRad);
            vRadialDbg      = vRadialFire;
            vLateralFireDbg = vLateralFire;

            // ── Step 3: Two-iteration solver for v_h_needed and d_eff ─────────
            // Iteration 1: rough tFlight via static setpoint at d_fire.
            ShooterSetpoint sp0   = ShooterKinematics.calculate(dFire);
            double tFlight        = ShooterKinematics.getFlightTimeSeconds(dFire, sp0, vRadialFire);
            double vRcomp         = dFire / tFlight - vRadialFire;
            double vHneeded       = Math.sqrt(vRcomp * vRcomp + vLateralFire * vLateralFire);
            double dEffCalc       = ShooterKinematics.distanceForHorizontalSpeed(vHneeded);
            ShooterSetpoint spEff = ShooterKinematics.calculate(dEffCalc);

            // Iteration 2: corrected tFlight using vx0 = vRcomp + vRadialFire.
            // The standard overload uses v_h + vRadialFire, but the correct radial
            // ball speed is v_h·cos(lead) + vRadialFire = vRcomp + vRadialFire.
            // At lead = 28° the difference is ~8% — non-negligible for tFlight.
            double vx0Total = vRcomp + vRadialFire;
            tFlight   = ShooterKinematics.getFlightTimeForHorizontalSpeed(vx0Total, spEff, dFire);
            vRcomp    = dFire / tFlight - vRadialFire;
            vHneeded  = Math.sqrt(vRcomp * vRcomp + vLateralFire * vLateralFire);
            dEffCalc  = ShooterKinematics.distanceForHorizontalSpeed(vHneeded);
            vx0Total  = vRcomp + vRadialFire; // update after refined vRcomp

            // ── Radial height correction ──────────────────────────────────────
            // RPM and hood are decoupled:
            //   RPM  ← dEffCalc (distanceForHorizontalSpeed) — satisfies lateral
            //           cancellation: v_h = vHneeded = sqrt(vRcomp² + vLateral²).
            //   Hood ← dEffHood  (distanceForGroundFrameShot) — satisfies height
            //           constraint: ball arrives at HUB_CENTER_HEIGHT at dFire
            //           with ground-frame radial speed vx0Total.
            //
            // For purely radial motion the old code gave dEffCalc ≈ dFire (no
            // RPM/hood change) because vRcomp ≈ v_h(dFire)·cos(θ).  Ball arrived
            // ~8 cm off-center at 2 m/s approach.  dEffHood corrects the vertical
            // component independently.
            double dEffHood = ShooterKinematics.distanceForGroundFrameShot(dFire, vx0Total);

            // Tuning scalar: 1.0 = full SOTM correction, 0.0 = static setpoint.
            dEffCalc = m_lastDistanceM + (dEffCalc - m_lastDistanceM) * Shooter.SOTM_RADIAL_SCALE;
            dEffCalc = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                       Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, dEffCalc));
            dEffHood = m_lastDistanceM + (dEffHood - m_lastDistanceM) * Shooter.SOTM_RADIAL_SCALE;
            dEffHood = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                       Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M, dEffHood));

            dEff = dEffCalc; // telemetry: report the lateral-constraint d_eff
            // Lateral-correct RPM + height-correct hood angle.
            setpoint = new ShooterSetpoint(
                ShooterKinematics.calculate(dEff).flywheelRPM(),
                ShooterKinematics.calculate(dEffHood).hoodAngleDeg()
            );
            SmartDashboard.putNumber("Shoot/DeffHoodM", dEffHood);

            // ── Step 4: Exact lead angle ──────────────────────────────────────
            // atan2 is exact and handles all quadrants including vRcomp < 0.
            leadAngleDeg = Math.toDegrees(Math.atan2(-vLateralFire, vRcomp))
                           * Shooter.SOTM_LEAD_ANGLE_SCALAR;

            SmartDashboard.putNumber("Shoot/DFireM",          dFire);
            SmartDashboard.putNumber("Shoot/AlphaFireDeg",    Math.toDegrees(alphaFireRad));
            SmartDashboard.putNumber("Shoot/AlphaNowDeg",     Math.toDegrees(alphaNowRad));
            SmartDashboard.putNumber("Shoot/VRadialFireMps",  vRadialFire);
            SmartDashboard.putNumber("Shoot/VLateralFireMps", vLateralFire);
            SmartDashboard.putNumber("Shoot/VHneededMps",     vHneeded);
        }

        // ── Turret command: r_pred at TURRET_PREDICTION_S ─────────────────────
        // TURRET_PREDICTION_S (0.10 s) = turret motor arrival time.
        // This block is computed independently from the physics block above.
        // Both use the same r_vector approach: atan2(predY, predX) gives the exact
        // hub direction including rotational AND translational pivot displacement.
        final double Tp = Turret.TURRET_PREDICTION_S;
        double[] turretTargetDeg = {Double.NaN};
        {
            double predX       = hubX - vxT * Tp - 0.5 * m_filtAxT * Tp * Tp;
            double predY       = hubY - vyT * Tp - 0.5 * m_filtAyT * Tp * Tp;
            double dPred       = Math.max(SuperstructureConstants.MIN_SHOOT_RANGE_M,
                                 Math.min(SuperstructureConstants.MAX_SHOOT_RANGE_M,
                                          Math.sqrt(predX * predX + predY * predY)));
            double alphaPredRad = Math.atan2(predY, predX);

            if (isWrapping) {
                // WRAPAROUND: point at hub direction only — no lead.
                // Suppressing lead during the wrap slew prevents the lead angle
                // itself from contributing to the out-of-bounds condition that
                // triggered WRAPAROUND in the first place.
                turretTargetDeg[0] = Math.toDegrees(alphaPredRad);
            } else {
                double vxTPred  = vxT + m_filtAxT * Tp;
                double vyTPred  = vyT + m_filtAyT * Tp;
                double vRpred   =  vxTPred * Math.cos(alphaPredRad) + vyTPred * Math.sin(alphaPredRad);
                double vLpred   = -vxTPred * Math.sin(alphaPredRad) + vyTPred * Math.cos(alphaPredRad);

                ShooterSetpoint spPred = ShooterKinematics.calculate(dPred);
                double tFlightPred     = ShooterKinematics.getFlightTimeSeconds(dPred, spPred, vRpred);
                double vRcompPred      = dPred / tFlightPred - vRpred;
                double leadPredDeg     = Math.toDegrees(Math.atan2(-vLpred, vRcompPred))
                                         * Shooter.SOTM_LEAD_ANGLE_SCALAR;

                turretTargetDeg[0] = Math.toDegrees(alphaPredRad) + leadPredDeg;
            }

            // Cable limit clamp — prevent spurious WRAPAROUND from lead overshoot.
            double finalEnc = -(turretTargetDeg[0] + Turret.TURRET_AIM_TRIM_DEG);
            if (finalEnc > Turret.TURRET_FORWARD_LIMIT_DEG)
                turretTargetDeg[0] = -Turret.TURRET_FORWARD_LIMIT_DEG - Turret.TURRET_AIM_TRIM_DEG;
            else if (finalEnc < Turret.TURRET_REVERSE_LIMIT_DEG)
                turretTargetDeg[0] = -Turret.TURRET_REVERSE_LIMIT_DEG - Turret.TURRET_AIM_TRIM_DEG;
        }

        // =====================================================================
        // PHASE 2 — SEND ALL SETPOINTS (flywheel, hood, turret) AT ONCE
        // =====================================================================

        m_superstructure.applyShooterSetpoint(setpoint);

        if (!Double.isNaN(turretTargetDeg[0])) {
            if (isWrapping) {
                m_superstructure.commandTurretAngle(turretTargetDeg[0]);
            } else {
                m_superstructure.commandTurretAngle(turretTargetDeg[0], vLateralFF, m_lastDistanceM);
            }
        }

        boolean distanceOK = isDistanceInRange();

        // --- Transition to SHOOTING when all conditions are satisfied --------
        boolean inPrepping = currentState == RobotState.PREPPING_TO_SHOOT;
        boolean isNearlyStationary = chassisSpeedMps < 0.1
                && Math.abs(rawSpd.omegaRadiansPerSecond) < 0.3;
        // Stationary: tight gate — all three mechanisms must be fully settled.
        // Moving: require isTrackingSetpoints() (flywheel + hood + turret all within
        // moving tolerances) so the turret has actually reached the lead angle before
        // the first ball fires.  Without this, the feeder starts immediately, the
        // turret is still at 0° (not at lead), and the shot misses.
        // The "delay" this introduces is correct and necessary — it is the time
        // for the turret to slew to the lead angle.  Once in SHOOTING the feeder
        // gate in handleShooting() keeps firing continuously.
        boolean mechanismsReady = isNearlyStationary
                ? m_superstructure.isReadyToShoot()
                : m_superstructure.isTrackingSetpoints();
        if (inPrepping && mechanismsReady && distanceOK) {
            m_superstructure.requestState(RobotState.SHOOTING);
        }

        // Drop back immediately if distance leaves range.
        if (currentState == RobotState.SHOOTING && !distanceOK) {
            if (++m_physicsNotOkCount >= PHYSICS_NOT_OK_DROP_LOOPS) {
                m_superstructure.requestState(RobotState.PREPPING_TO_SHOOT);
                m_physicsNotOkCount = 0;
            }
        } else {
            m_physicsNotOkCount = 0;
        }

        // --- Diagnostic telemetry (visible on SmartDashboard) ----------------
        SmartDashboard.putBoolean("Shoot/InPrepping",         inPrepping);
        SmartDashboard.putBoolean("Shoot/MechanismsReady",    mechanismsReady);
        SmartDashboard.putBoolean("Shoot/DistanceInRange",    distanceOK);
        SmartDashboard.putBoolean("Shoot/IsStationary",       isStationary);
        SmartDashboard.putNumber( "Shoot/DistanceM",          m_lastDistanceM);
        SmartDashboard.putNumber( "Shoot/RawDistanceM",       m_rawDistanceM);
        SmartDashboard.putNumber( "Shoot/DeffM",              dEff);
        SmartDashboard.putNumber( "Shoot/VRadialNowMps",      vRadialDbg);
        SmartDashboard.putNumber( "Shoot/VLateralFireMps",    vLateralFireDbg);
        SmartDashboard.putNumber( "Shoot/LeadAngleDeg",       leadAngleDeg);
        SmartDashboard.putNumber( "Shoot/AccelX",             m_filtAxT);
        SmartDashboard.putNumber( "Shoot/AccelY",             m_filtAyT);
        SmartDashboard.putNumber( "Shoot/OmegaRadPerSec",     omega);
        // Ball exit angle (= 90° − hood angle) — verify this matches what you
        // physically observe from the robot (slow-motion camera or angle gauge).
        // If it does NOT match, the hoodToBallExitAngleDeg formula is wrong.
        SmartDashboard.putNumber( "Shoot/BallExitAngleDeg", 90.0 - setpoint.hoodAngleDeg());
        SmartDashboard.putNumber( "Shoot/FlywheelRPMCmd",   setpoint.flywheelRPM());
        SmartDashboard.putNumber( "Shoot/HoodAngleCmd",     setpoint.hoodAngleDeg());
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

    // =========================================================================
    // Private Helpers
    // =========================================================================

    /**
     * Returns whether the current robot-to-hub distance is within the shootable range.
     *
     * <p>Uses {@code m_rawDistanceM} (the actual measurement this loop) when vision
     * provided a fresh reading — this correctly blocks shots when the robot drifts
     * outside the range.  Falls back to {@code m_lastDistanceM} only when no
     * vision measurement arrived this loop (e.g. brief tag loss while tracking).
     */
    private boolean isDistanceInRange() {
        double d = m_rawDistanceValid ? m_rawDistanceM : m_lastDistanceM;
        return d >= SuperstructureConstants.MIN_SHOOT_RANGE_M
            && d <= SuperstructureConstants.MAX_SHOOT_RANGE_M;
    }
}
