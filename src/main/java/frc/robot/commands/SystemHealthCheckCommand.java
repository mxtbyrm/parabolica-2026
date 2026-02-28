package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.Shooter;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.RobotState;

/**
 * Sequentially exercises every scoring mechanism to verify motor connectivity
 * and basic closed-loop response.  Results are published to SmartDashboard
 * under the {@code "Health/"} prefix as individual booleans, plus an overall
 * {@code "Health/Status"} string ({@code "PASS"} or {@code "FAIL — see above"}).
 *
 * <h2>Tests performed (in order)</h2>
 * <ol>
 *   <li><b>No Faults</b>  — {@code hasCriticalFault()} false on all subsystems</li>
 *   <li><b>Flywheel</b>   — spins to 500 RPM; confirmed reaching &gt;400 RPM</li>
 *   <li><b>Hood</b>       — moves to midpoint angle; confirmed within tolerance</li>
 *   <li><b>Turret</b>     — slews to +15°; confirmed within tolerance; returns to 0°</li>
 *   <li><b>Feeder</b>     — runs 0.5 s; stator current &gt;1 A confirms motor connectivity</li>
 *   <li><b>Spindexer</b>  — same as feeder</li>
 *   <li><b>Intake Deploy</b> — arm moves to deployed position; confirmed via {@code isDeployed()}</li>
 *   <li><b>Intake Roller</b> — fault check while arm is deployed (roller runs during deploy)</li>
 * </ol>
 *
 * <p><b>Robot must be enabled</b> — motor commands are suppressed by the RoboRIO
 * while disabled.  Register this command on SmartDashboard and click the button
 * while the robot is in Teleop or Test mode.
 *
 * <p>Requires all scoring subsystems and the Superstructure; interrupts any
 * active scoring command for the duration of the check (~12 s).
 */
public class SystemHealthCheckCommand extends SequentialCommandGroup {

    private static final String PFX = "Health/";

    public SystemHealthCheckCommand(
            ShooterSubsystem   shooter,
            TurretSubsystem    turret,
            FeederSubsystem    feeder,
            SpindexerSubsystem spindexer,
            IntakeSubsystem    intake,
            Superstructure     superstructure) {

        // Track overall pass/fail across all lambdas.
        // boolean[] is effectively-final (reference doesn't change, contents do).
        boolean[] pass = {true};

        addCommands(

            // -----------------------------------------------------------------
            // Init — clear previous results, move Superstructure to safe state
            // -----------------------------------------------------------------
            Commands.runOnce(() -> {
                pass[0] = true;
                SmartDashboard.putString( PFX + "Status",       "Running...");
                SmartDashboard.putBoolean(PFX + "No Faults",    false);
                SmartDashboard.putBoolean(PFX + "Flywheel",     false);
                SmartDashboard.putBoolean(PFX + "Hood",         false);
                SmartDashboard.putBoolean(PFX + "Turret",       false);
                SmartDashboard.putBoolean(PFX + "Feeder",       false);
                SmartDashboard.putBoolean(PFX + "Spindexer",    false);
                SmartDashboard.putBoolean(PFX + "Intake Deploy",false);
                SmartDashboard.putBoolean(PFX + "Intake Roller",false);
                superstructure.requestState(RobotState.STOWED);
            }, superstructure),

            // -----------------------------------------------------------------
            // 1. Fault check — all subsystems
            // -----------------------------------------------------------------
            Commands.runOnce(() -> {
                boolean ok = !shooter.hasCriticalFault()
                          && !turret.hasCriticalFault()
                          && !feeder.hasCriticalFault()
                          && !spindexer.hasCriticalFault()
                          && !intake.hasCriticalFault();
                SmartDashboard.putBoolean(PFX + "No Faults", ok);
                if (!ok) pass[0] = false;
            }, shooter, turret, feeder, spindexer, intake),

            // -----------------------------------------------------------------
            // 2. Flywheel — spin to 500 RPM, confirm ≥ 400 RPM after 2 s
            // -----------------------------------------------------------------
            Commands.runOnce(() -> shooter.setFlywheelRPM(500.0), shooter),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> {
                boolean ok = shooter.getFlywheelRPM() >= 400.0;
                SmartDashboard.putBoolean(PFX + "Flywheel", ok);
                if (!ok) pass[0] = false;
            }, shooter),
            Commands.runOnce(shooter::stopFlywheel, shooter),

            // -----------------------------------------------------------------
            // 3. Hood — move to midpoint, wait for arrival, return to min
            // -----------------------------------------------------------------
            Commands.runOnce(() -> shooter.setHoodAngle(
                    (Shooter.HOOD_MIN_ANGLE_DEG + Shooter.HOOD_MAX_ANGLE_DEG) / 2.0), shooter),
            Commands.waitUntil(shooter::isHoodAtAngle).withTimeout(3.0),
            Commands.runOnce(() -> {
                boolean ok = shooter.isHoodAtAngle();
                SmartDashboard.putBoolean(PFX + "Hood", ok);
                if (!ok) pass[0] = false;
            }, shooter),
            Commands.runOnce(() -> shooter.setHoodAngle(Shooter.HOOD_MIN_ANGLE_DEG), shooter),
            Commands.waitUntil(shooter::isHoodAtAngle).withTimeout(3.0),

            // -----------------------------------------------------------------
            // 4. Turret — slew to +15°, confirm, return to 0°
            // -----------------------------------------------------------------
            Commands.runOnce(() -> turret.setAngle(15.0), turret),
            Commands.waitUntil(turret::isAligned).withTimeout(3.0),
            Commands.runOnce(() -> {
                boolean ok = turret.isAligned();
                SmartDashboard.putBoolean(PFX + "Turret", ok);
                if (!ok) pass[0] = false;
            }, turret),
            Commands.runOnce(() -> turret.setAngle(0.0), turret),
            Commands.waitUntil(turret::isAligned).withTimeout(3.0),

            // -----------------------------------------------------------------
            // 5. Feeder — run 0.5 s, verify stator current > 1 A
            // -----------------------------------------------------------------
            Commands.runOnce(feeder::feed, feeder),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                boolean ok = feeder.getStatorCurrentAmps() > 1.0 && !feeder.hasCriticalFault();
                SmartDashboard.putBoolean(PFX + "Feeder", ok);
                if (!ok) pass[0] = false;
            }, feeder),
            Commands.runOnce(feeder::stop, feeder),

            // -----------------------------------------------------------------
            // 6. Spindexer — same as feeder
            // -----------------------------------------------------------------
            Commands.runOnce(spindexer::run, spindexer),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                boolean ok = spindexer.getStatorCurrentAmps() > 1.0 && !spindexer.hasCriticalFault();
                SmartDashboard.putBoolean(PFX + "Spindexer", ok);
                if (!ok) pass[0] = false;
            }, spindexer),
            Commands.runOnce(spindexer::stop, spindexer),

            // -----------------------------------------------------------------
            // 7. Intake deploy — arm to deployed, confirm isDeployed()
            // 8. Intake roller — runs during deploy; check fault while deployed
            // -----------------------------------------------------------------
            Commands.runOnce(intake::deploy, intake),
            Commands.waitUntil(intake::isDeployed).withTimeout(4.0),
            Commands.runOnce(() -> {
                boolean deployOk = intake.isDeployed();
                boolean rollerOk = !intake.hasCriticalFault(); // covers all 3 intake motors
                SmartDashboard.putBoolean(PFX + "Intake Deploy", deployOk);
                SmartDashboard.putBoolean(PFX + "Intake Roller", rollerOk);
                if (!deployOk || !rollerOk) pass[0] = false;
            }, intake),
            Commands.runOnce(intake::stow, intake),
            Commands.waitSeconds(1.5), // allow arm to clear before re-enabling normal ops

            // -----------------------------------------------------------------
            // Final summary
            // -----------------------------------------------------------------
            Commands.runOnce(() ->
                SmartDashboard.putString(PFX + "Status",
                        pass[0] ? "PASS" : "FAIL — check above"))
        );
    }
}
