package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class StowTemp implements EnterableState {
  /**
   * Stows all the stowables on the robot
   */
  public StowTemp() {}

  public Command build(Subsystems subsystems) {
    return Commands.parallel(
        subsystems.elevator().go(ElevatorState.Stow),
        subsystems.pivot().go(PivotState.Stow))
        
        .finallyDo(subsystems.elevator()::release) // This allows the elevator to stow itself automatically.
        .finallyDo(subsystems.pivot()::release);
  }
}

