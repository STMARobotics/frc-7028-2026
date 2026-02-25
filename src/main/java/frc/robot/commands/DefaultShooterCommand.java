package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.PITCH_HOME_ANGLE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Default command for the shooter. Holds the hood at its home position and stops the motor if within deadband. Turns
 * off yaw and flywheel.
 */
public class DefaultShooterCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;

  public DefaultShooterCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.stopFlywheel();
    shooterSubsystem.stopYaw();
  }

  @Override
  public void execute() {
    if (shooterSubsystem.isPitchStowed()) {
      shooterSubsystem.stopPitch();
    } else {
      shooterSubsystem.setPitchAngle(PITCH_HOME_ANGLE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopAll();
  }
}
