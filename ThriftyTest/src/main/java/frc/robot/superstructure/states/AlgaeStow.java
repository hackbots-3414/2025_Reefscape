package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class AlgaeStow implements EnterableState {
  /**
   * A state that holds an algae in a disable-safe position. What this means is that if the robot is
   * disabled while in this state, an algae present will not fall out.
   */
  public AlgaeStow() {}

  public Command build(Subsystems subsystems) {
    return Commands.parallel(
        subsystems.elevator().go(ElevatorState.Stow).asProxy(),
        subsystems.pivot().ground())

        .onlyIf(subsystems.algae().holdingAlgae());
    // Neither subsystem is released because this is a "persistent" state.
  }
}
