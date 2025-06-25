package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class LowerReefAlgaeIntake implements EnterableState {
  /**
   * A state to intake the algae off of the reef
   */
  public LowerReefAlgaeIntake() {}

  public Command build(Subsystems subsystems) {
    return Commands.sequence(
        subsystems.elevator().go(ElevatorState.LowerReef),
        Commands.parallel(
            subsystems.pivot().go(PivotState.ReefIntake),
            subsystems.algae().intake()),
        subsystems.pivot().go(PivotState.ReefExtract))

        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.algae()::release)
        .finallyDo(subsystems.pivot()::release)
        .unless(subsystems.algae().holdingAlgae());
  }
}
