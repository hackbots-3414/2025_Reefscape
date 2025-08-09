package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class ClimbStowed implements EnterableState {
  /**
   * A state that represents the climber at its "stow" position
   */
  public ClimbStowed() {}

  public Command build(Subsystems subsystems) {
    return subsystems.climber().lower();
  }
}
