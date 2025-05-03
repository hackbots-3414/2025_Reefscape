package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class FunnelOpened implements EnterableState {
  /**
   * A state that represents the funnel in the open state
   */
  public FunnelOpened() {}

  public Command build(Subsystems subsystems) {
    return subsystems.climber().openFunnel();
  }
}
