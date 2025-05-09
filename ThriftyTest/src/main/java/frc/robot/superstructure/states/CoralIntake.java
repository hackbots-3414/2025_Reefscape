package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CoralIntake implements EnterableState {
  /**
   * A state to intake coral
   */
  public CoralIntake() {}

  public Command build(Subsystems subsystems) {
    return Commands.sequence(
        subsystems.elevator().autoZero(),
        subsystems.elevator().go(ElevatorState.Stow),
        subsystems.coral().intake())

        .unless(subsystems.coral().holding())
        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.coral()::release);

  }
}
