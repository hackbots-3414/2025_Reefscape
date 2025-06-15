package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class GroundAlgaeIntake implements EnterableState {
  /**
   * A state to intake algae from the ground
   */
  public GroundAlgaeIntake() {}

  public Command build(Subsystems subsystems) {
    return Commands.parallel(
        subsystems.elevator().go(ElevatorState.Ground).asProxy(),
        subsystems.pivot().go(PivotState.Ground),
        subsystems.algae().intake())

        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.pivot()::release)
        .finallyDo(subsystems.algae()::release)
        .unless(subsystems.algae().holdingAlgae());
  }
}
