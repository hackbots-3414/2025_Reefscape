package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class ElevatorZero implements EnterableState {
  /**
   * A state that calibrates the elevator to its zero position
   */
  public ElevatorZero() {}

  public Command build(Subsystems subsystems) {
    return subsystems.elevator().autoZero().asProxy()
        .finallyDo(subsystems.elevator()::release)
        .unless(subsystems.coral().present());
  }
}
