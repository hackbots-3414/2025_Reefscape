package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class HighGroundAlgaeIntake implements EnterableState {
  /**
   * A state to intake algae off the "lollipops"
   */
  public HighGroundAlgaeIntake() {}

  public Command build(Subsystems subsystems) {
    return Commands.parallel(
        subsystems.elevator().go(ElevatorState.UpperReef),
        subsystems.pivot().ground(),
        subsystems.algae().intake())

        .unless(subsystems.algae().holdingAlgae())
        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.pivot()::release);
  }
}
