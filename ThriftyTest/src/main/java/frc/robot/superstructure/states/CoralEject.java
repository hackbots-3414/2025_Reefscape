package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CoralEject implements EnterableState {
  /**
   * A state that represents a present coral being ejected
   */
  public CoralEject() {}

  public Command build(Subsystems subsystems) {
    return Commands.sequence(
        subsystems.elevator().go(ElevatorState.Eject),
        subsystems.coral().eject())

        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.coral()::release)
        .onlyIf(subsystems.coral().present());
  }
}
