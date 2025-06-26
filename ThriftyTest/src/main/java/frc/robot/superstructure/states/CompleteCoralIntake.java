package frc.robot.superstructure.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class CompleteCoralIntake implements EnterableState {
  /**
   * A state to complete a coral intake if it has been started <br>
   * <br>
   * This is different from the passive behavior on the coral subsystem itself because it also
   * requires the elevator, which makes it more of a "blocking" operation.
   */
  public CompleteCoralIntake() {}

  public Command build(Subsystems subsystems) {
    return new CoralIntake().build(subsystems)
        .finallyDo(subsystems.coral()::release)
        .onlyIf(subsystems.coral().present());
  }
}
