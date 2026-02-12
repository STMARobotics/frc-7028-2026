package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TransferSubsystem;

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
    shooterSubsystem.applySetpoints(null);
    if (shooterSubsystem.isFlywheelAtSpeed()) {
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
