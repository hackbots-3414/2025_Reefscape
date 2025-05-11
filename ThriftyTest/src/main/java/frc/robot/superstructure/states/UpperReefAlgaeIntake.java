package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.pivot.PivotState;
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

            subsystems.pivot().go(PivotState.ReefIntake),
            subsystems.algae().intake()),
        subsystems.pivot().go(PivotState.ReefExtract))

        .finallyDo(subsystems.pivot()::release)
        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.algae()::release)
        .unless(subsystems.algae().holdingAlgae());
  }
}
