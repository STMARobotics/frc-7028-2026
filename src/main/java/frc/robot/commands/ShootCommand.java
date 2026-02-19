package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

/*
 * Command to shoot fuel without aiming. It will shoot at a fixed yaw, pitch, and velocity
 */
public class ShootCommand extends Command {
  private final SpindexerSubsystem spindexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  /**
   * Constructor for ShootCommand
   * 
   * @param spindexerSubsystem the spindexer subsystem
   * @param feederSubsystem the feeder subsystem
   * @param shooterSubsystem the shooter subsystem
   */
  public ShootCommand(
      SpindexerSubsystem spindexerSubsystem,
      FeederSubsystem feederSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.feederSubsystem = feederSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(feederSubsystem, spindexerSubsystem, shooterSubsystem);
  }

  @Override
  public void execute() {
    /*
     * sets the Yaw, Pitch, and Angle
     */
    shooterSubsystem.setYawAngle(Constants.ShooterConstants.YAW_HOME_ANGLE);
    shooterSubsystem.setPitchAngle(Constants.ShooterConstants.PITCH_HOME_ANGLE);
    shooterSubsystem.setFlywheelSpeed(RotationsPerSecond.of(40));
    /*
     * Checks to make sure the shooter is ready and up to speed
     * before runnig the spindexer and feeder
     */
    if (shooterSubsystem.isReadyToShoot()) {
      spindexerSubsystem.feedShooter();
      feederSubsystem.feedShooter();
    }
  }

  public void end(boolean interrupted) {
    spindexerSubsystem.stop();
    feederSubsystem.stop();
    shooterSubsystem.stopAll();
  }
}
