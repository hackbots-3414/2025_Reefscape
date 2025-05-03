package frc.robot.superstructure.states;

import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class Aligned implements EnterableState {
  private final Pose2d m_goal;
  /**
   * A state that controls the drivetrain and drives to a certain, given point on the field.
   */
  public Aligned(Pose2d goal) {
    m_goal = goal;
  }

  public Command build(Subsystems subsystems) {
    return subsystems.drivetrain().align(m_goal);
  }
}
