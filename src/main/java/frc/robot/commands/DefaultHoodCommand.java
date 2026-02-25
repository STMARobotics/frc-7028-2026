package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.PITCH_HOME_ANGLE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Default command for the hood. Holds the hood at its home position and stops the motor if within deadband.
 */
public class DefaultHoodCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;

  public DefaultHoodCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    if (shooterSubsystem.isPitchStowed()) {
      shooterSubsystem.stopPitch();
    } else {
      shooterSubsystem.setPitchAngle(PITCH_HOME_ANGLE);
    }
  }
}
