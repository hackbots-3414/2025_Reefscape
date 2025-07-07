package frc.robot.superstructure.states;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class Test implements EnterableState {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(Test.class);

  /**
   * A test state
   */
  public Test() {}

  public Command build(Subsystems subsystems) {
    return Commands.none();
  }
}
