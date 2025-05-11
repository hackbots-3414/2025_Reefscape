package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CoralIntake implements EnterableState {
  /**
   * A state to intake coral
   */
  public CoralIntake() {}

  public Command build(Subsystems subsystems) {
    return Commands.sequence(
        subsystems.elevator().autoZero().asProxy(),
        subsystems.elevator().go(ElevatorState.Stow).asProxy(),
        subsystems.coral().intake())

        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.coral()::release)
        .unless(subsystems.coral().holding());

  }
}
