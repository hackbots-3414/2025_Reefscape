package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class ProcessorPrep implements EnterableState {
  /**
   * A state that prepares the robot for a processor score
   */
  public ProcessorPrep() {}

  public Command build(Subsystems subsystems) {
    return Commands.parallel(
        subsystems.elevator().go(ElevatorState.Processor).asProxy(),
        subsystems.pivot().processor())

        .finallyDo(subsystems.elevator()::conditionalRelease)
        .finallyDo(subsystems.pivot()::conditionalRelease)
        .onlyIf(subsystems.algae().holdingAlgae());
  }
}
