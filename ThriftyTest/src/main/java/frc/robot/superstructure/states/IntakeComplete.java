package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class IntakeComplete implements EnterableState {
  /**
   * A state that waits for a coral to complete intaking as long as there is a coral in the robot
   *
   * This command will end if a coral is no longer detected.
   */
  public IntakeComplete() {}

  public Command build(Subsystems subsystems) {
    return Commands.waitUntil(subsystems.coral().held())
        .onlyWhile(subsystems.coral().present());
  }
}
