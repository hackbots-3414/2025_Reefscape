package frc.robot.binding;

import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.modifiers.AlgaeElevatorHold;
import frc.robot.superstructure.modifiers.ElevatorPrep;

public class RobotBindings implements Binder {
  public RobotBindings() {}

  public void bind(Superstructure superstructure) {
    /* elevator prefire */
    superstructure.modify(new ElevatorPrep(),
        superstructure.inReefZone().and(superstructure.holdingCoral()));
    /* no elevator movement when in reef and holding algae */
    superstructure.modify(new AlgaeElevatorHold(),
        superstructure.inReefZone().and(superstructure.holdingAlgae()));
  }
}
