package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * Command to climb to L1 (Level 1) in autonomous mode.
 * 
 * <p>
 * This command prepares the climb mechanism and pulls the robot up to the L1 bar.
 */
public class ClimbToL1Command extends Command {

  private final ClimbSubsystem climbSubsystem;

  /**
   * Creates a new ClimbToL1Command.
   * 
   * @param climbSubsystem the climb subsystem
   */
  public ClimbToL1Command(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.prepareToL1();
    climbSubsystem.run(climbSubsystem::prepareToL1)
        .until(climbSubsystem::isActionComplete)
        .andThen(climbSubsystem::L1)
        .until(climbSubsystem::isActionComplete)
        .finallyDo(climbSubsystem::stop);
  }

  @Override
  public void execute() {
    // Wait until the prepare action is complete, then move to L1
    if (climbSubsystem.getCurrentState() == ClimbSubsystem.ClimbState.PREPARE_TO_L1
        && climbSubsystem.isActionComplete()) {
      climbSubsystem.L1();
    }
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Finished when we're at L1 and the action is complete
    return climbSubsystem.getCurrentState() == ClimbSubsystem.ClimbState.L1 && climbSubsystem.isActionComplete();
  }
}
