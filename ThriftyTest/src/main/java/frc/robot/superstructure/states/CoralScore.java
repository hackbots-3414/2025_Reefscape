package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CoralLevel;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CoralScore implements EnterableState {
  private CoralLevel m_level;

  /**
   * A state that scores a coral on a specified level on the reef.
   */
  public CoralScore(CoralLevel level) {
    m_level = level;
  }

  public Command build(Subsystems subsystems) {
    return Commands.sequence(
        subsystems.elevator().go(m_level),
        subsystems.coral().score(m_level))

        .onlyIf(subsystems.coral().holding())
        .finallyDo(subsystems.elevator()::release)
        .finallyDo(subsystems.coral()::release);
  }
}
