package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class ProcessorReady implements EnterableState {
  /**
   * A state that prepares the robot for a processor score
   */
  public ProcessorReady() {}

  public Command build(Subsystems subsystems) {
    return Commands.parallel(
        subsystems.elevator().go(ElevatorState.Processor).asProxy(),
        subsystems.pivot().processor())

        .onlyIf(subsystems.algae().holdingAlgae())
        .finallyDo(subsystems.elevator()::conditionalRelease)
        .finallyDo(subsystems.pivot()::conditionalRelease);
  }
}
