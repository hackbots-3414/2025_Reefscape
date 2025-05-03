package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralLevel;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CoralScoreReady implements EnterableState {
  private final CoralLevel m_level;

  /**
   * A state that prepares the robot to score at the desired level
   */
  public CoralScoreReady(CoralLevel level) {
    m_level = level;
  }

  public Command build(Subsystems subsystems) {
    return subsystems.elevator().go(m_level)
      .onlyIf(subsystems.coral().holding())
      .finallyDo(subsystems.elevator()::conditionalRelease);
  }
}
