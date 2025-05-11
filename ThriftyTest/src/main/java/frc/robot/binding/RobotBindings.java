package frc.robot.binding;

import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.modifiers.ElevatorPrep;

public class RobotBindings implements Binder {
  public RobotBindings() {}

  public void bind(Superstructure superstructure) {
    /* elevator prefire */
    superstructure.modify(new ElevatorPrep(),
        superstructure.inReefZone().and(superstructure.holdingCoral()));
  }
}
