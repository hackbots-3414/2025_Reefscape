package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CompleteCoralIntake implements EnterableState {
  /**
   * A state to complete a coral intake if it can be done
   */
  public CompleteCoralIntake() {}

  public Command build(Subsystems subsystems) {
    return new CoralIntake().build(subsystems)
        .onlyIf(subsystems.coral().present());
  }
}
