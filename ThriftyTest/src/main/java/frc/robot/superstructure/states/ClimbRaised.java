package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class ClimbRaised implements EnterableState {
  /**
   * A state with the climber at the raised position
   */
  public ClimbRaised() {}

  public Command build(Subsystems subsystems) {
    return subsystems.climber().raise();
  }
}
