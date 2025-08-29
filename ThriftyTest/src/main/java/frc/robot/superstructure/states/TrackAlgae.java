package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class TrackAlgae implements EnterableState {
  public Command build(Subsystems subsystems) {
    return subsystems.drivetrain().followObject();
  }
}
