package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class HighGroundAlgaeIntake implements EnterableState {
  /**
   * A state to intake algae off the "lollipops"
   */
  public HighGroundAlgaeIntake() {}

  public Command build(Subsystems subsystems) {
    return Commands.sequence(
        subsystems.elevator().go(ElevatorState.HighGround),
        Commands.parallel(
            subsystems.pivot().go(PivotState.HighGround),
            subsystems.algae().intake()))

        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.pivot()::release)
        .finallyDo(subsystems.algae()::release)
        .unless(subsystems.algae().holdingAlgae());
  }
}

