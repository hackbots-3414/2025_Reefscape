package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralLevel;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CompleteCoralScore implements EnterableState {
  private final CoralLevel m_level;

  /**
   * A state that completes a coral intake cycle, if the robot is actively ready.
   */
  public CompleteCoralScore(CoralLevel level) {
    m_level = level;
  }

  public Command build(Subsystems subsystems) {
    return new CoralScore(m_level).build(subsystems)
      .onlyIf(subsystems.elevator().ready(m_level));
  }
}
