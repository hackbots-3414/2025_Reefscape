package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class Net implements EnterableState {
  /**
   * A state that scores an algae in the net
   */
  public Net() {}

  public Command build(Subsystems subsystems) {
    return Commands.sequence(
        Commands.parallel(
            subsystems.elevator().go(ElevatorState.Net),
            subsystems.pivot().net()),
        subsystems.algae().net())

        .onlyIf(subsystems.algae().holdingAlgae())
        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.pivot()::release);
  }
}
