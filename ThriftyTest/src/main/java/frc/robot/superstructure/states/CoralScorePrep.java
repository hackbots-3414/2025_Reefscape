package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralLevel;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CoralScorePrep implements EnterableState {
  private final CoralLevel m_level;

  /**
   * A state that prepares the robot to score at the desired level
   */
  public CoralScorePrep(CoralLevel level) {
    m_level = level;
  }

  public Command build(Subsystems subsystems) {
    return subsystems.elevator().go(m_level)
        .finallyDo(subsystems.elevator()::conditionalRelease)
        .onlyIf(subsystems.coral().held());
  }
}
