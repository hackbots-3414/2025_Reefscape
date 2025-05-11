package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class NetReady implements EnterableState {
  /**
   * A state that sets the robot up to score in the net
   */
  public NetReady() {}

  public Command build(Subsystems subsystems) {
    return Commands.parallel(
        subsystems.elevator().go(ElevatorState.Net).asProxy(),
        subsystems.pivot().go(PivotState.Net))

        .finallyDo(subsystems.elevator()::conditionalRelease)
        .finallyDo(subsystems.pivot()::conditionalRelease)
        .onlyIf(subsystems.algae().holdingAlgae());
  }
}
