package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.SpindexerSubsystem;

/**
 * Command to intake fuel from the floor. This has a prerequisite of the intake being deployed.
 */
public class IntakeCommand extends Command {

  private final IntakeSubsytem intakeSubsytem;
  private final SpindexerSubsystem spindexerSubsystem;

  public IntakeCommand(IntakeSubsytem intakeSubsytem, SpindexerSubsystem spindexerSubsystem) {
    this.intakeSubsytem = intakeSubsytem;
    this.spindexerSubsystem = spindexerSubsystem;

    addRequirements(intakeSubsytem, spindexerSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsytem.runIntake();
    spindexerSubsystem.intake();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsytem.stopIntaking();
    spindexerSubsystem.stop();
  }

}
