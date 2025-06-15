package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class HeadingReset implements EnterableState {
  /**
   * An instant state that resets the operator forward perspective for teleop control
   */
  public HeadingReset() {}

  public Command build(Subsystems subsystems) {
    return subsystems.drivetrain().resetHeading();
  }
}
