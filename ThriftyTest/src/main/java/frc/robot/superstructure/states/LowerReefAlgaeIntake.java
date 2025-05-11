package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class LowerReefAlgaeIntake implements EnterableState {
  /**
   * A state to intake the algae off of the reef
   */
  public LowerReefAlgaeIntake() {}

  public Command build(Subsystems subsystems) {
    return Commands.sequence(
        Commands.parallel(
            subsystems.elevator().go(ElevatorState.LowerReef).asProxy(),
            subsystems.pivot().reefIntake(),
            subsystems.algae().intake()),
        subsystems.pivot().reefExtract())

        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.pivot()::release)
        .unless(subsystems.algae().holdingAlgae());
  }
}
