package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.Superstructure.Subsystems;

/**
 * A robot has a lot of possible states. This class represents a single one.
 *
 * A state is a possible configuration of the robot. The EnterableState has a method
 * <code>build(subsystems)</code> which builds a command that, if possible, directs the robot to
 * enter the state that the class represents.
 */
public interface EnterableState {
  /**
   * Builds a command to enter this state.
   */
  Command build(Subsystems subsystems);
}
