package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.run;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * Command to climb to L3 (Level 3).
 */
public class ClimbToL3Command extends SequentialCommandGroup {

  /**
   * Creates a new ClimbToL3Command.
   * 
   * @param climbSubsystem the climb subsystem
   */
  public ClimbToL3Command(ClimbSubsystem climbSubsystem) {
    addCommands(
        run(() -> climbSubsystem.prepareToL1(), climbSubsystem).until(climbSubsystem::isActionComplete),
          run(() -> climbSubsystem.L1(), climbSubsystem).until(climbSubsystem::isActionComplete),
          run(() -> climbSubsystem.prepareToL2(), climbSubsystem).until(climbSubsystem::isActionComplete),
          run(() -> climbSubsystem.L2(), climbSubsystem).until(climbSubsystem::isActionComplete),
          run(() -> climbSubsystem.prepareToL3(), climbSubsystem).until(climbSubsystem::isActionComplete),
          run(() -> climbSubsystem.L3(), climbSubsystem).until(climbSubsystem::isActionComplete));
  }
}
