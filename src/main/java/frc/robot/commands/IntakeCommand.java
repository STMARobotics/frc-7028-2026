package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;

/**
 * Command to intake fuel from the floor. This has a prerequisite of the intake being deployed.
 */
public class IntakeCommand extends Command {

  private final IntakeSubsytem intakeSubsytem;

  public IntakeCommand(IntakeSubsytem intakeSubsytem) {
    this.intakeSubsytem = intakeSubsytem;

    addRequirements(intakeSubsytem);
  }

  @Override
  public void initialize() {
    intakeSubsytem.runIntake();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsytem.stopIntaking();
  }

}
