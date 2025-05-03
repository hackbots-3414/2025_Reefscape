package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class UpperReefAlgaeIntake implements EnterableState {
  /**
   * A state that intakes algae off of the higher level of the reef
   */
  public UpperReefAlgaeIntake() {}

  public Command build(Subsystems subsystems) {
    return Commands.sequence(
        Commands.parallel(
            subsystems.elevator().go(ElevatorState.UpperReef).asProxy(),

            subsystems.pivot().reefIntake(),
            subsystems.algae().intake()),
        subsystems.pivot().reefExtract())

        .unless(subsystems.algae().holdingAlgae())
        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.pivot()::release);
  }
}
