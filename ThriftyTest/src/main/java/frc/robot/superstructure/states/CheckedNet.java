package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CheckedNet implements EnterableState {
  /**
   * A state that scores an algae in the net
   */
  public CheckedNet() {}

  public Command build(Subsystems subsystems) {
    return new UncheckedNet().build(subsystems)
        .onlyIf(subsystems.elevator().ready(ElevatorState.Net));
  }
}
