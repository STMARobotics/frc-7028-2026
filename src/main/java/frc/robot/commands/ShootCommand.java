package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TransferSubsystem;

/*
 * Command to shoot balls from the Spindexer
 */

public class ShootCommand extends Command {
  private final SpindexerSubsystem spindexerSubsystem;
  private final TransferSubsystem transferSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ShootCommand(
      SpindexerSubsystem spindexerSubsystem,
      TransferSubsystem transferSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.transferSubsystem = transferSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(transferSubsystem, spindexerSubsystem, shooterSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.isReadyToShoot();
    /*
     * sets the Yaw, Pitch, and Angle
     */
    shooterSubsystem.setYawAngle(Constants.ShooterConstants.YAW_HOME_ANGLE);
    shooterSubsystem.setPitchAngle(Constants.ShooterConstants.PITCH_HOME_ANGLE);
    shooterSubsystem.setFlywheelSpeed(RotationsPerSecond.of(40));
    /*
     * Checks to make sure the shooter is ready and up to speed
     * before runnig the spindexer and transfer
     */
    if (shooterSubsystem.isReadyToShoot()) {
      spindexerSubsystem.feedShooter();
      transferSubsystem.feedShooter();
    }
  }

  public void end(boolean interrupted) {
    spindexerSubsystem.stop();
    transferSubsystem.stop();
    shooterSubsystem.stopAll();
  }
}
