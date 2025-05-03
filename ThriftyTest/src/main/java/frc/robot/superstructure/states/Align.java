package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.driveassist.Autopilot;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class Align implements EnterableState {
  private final Autopilot m_autopilot;
  private final Autopilot.Target m_target;

  /**
   * A state that controls the drivetrain and drives to a certain, given point on the field.
   */
  public Align(Autopilot autopilot, Autopilot.Target target) {
    m_autopilot = autopilot;
    m_target = target;
  }

  public Command build(Subsystems subsystems) {
    return subsystems.drivetrain().align(m_autopilot, m_target);
  }
}
