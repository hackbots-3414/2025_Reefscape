package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CoralWait implements EnterableState {
  /**
   * A state that puts the robot into a "waiting" state until a coral can be acquired.
   * The state is NOT a coral itake state, however, and does not actually change any mechanisms
   */
  public CoralWait() {}

  public Command build(Subsystems subsystems) {
    return Commands.waitUntil(subsystems.coral().present());
  }
}
