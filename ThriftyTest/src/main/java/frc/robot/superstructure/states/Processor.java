package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class Processor implements EnterableState {
  /**
   * A state that scores an algae in the processor
   */
  public Processor() {}

  public Command build(Subsystems subsystems) {
    return Commands.sequence(
        Commands.parallel(
            subsystems.elevator().go(ElevatorState.Processor).asProxy(),
            subsystems.pivot().go(PivotState.Processor)),
        subsystems.algae().processorScore())

        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.pivot()::release)
        .finallyDo(subsystems.algae()::release)
        .onlyIf(subsystems.algae().holdingAlgae());
  }
}
