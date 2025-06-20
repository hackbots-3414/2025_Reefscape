package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class Climb implements EnterableState {
  /**
   * A state in which the robot has lowered the climber to the climb setpoint
   */
  public Climb() {}

  public Command build(Subsystems subsystems) {
    return subsystems.climber().climb();
  }
}
