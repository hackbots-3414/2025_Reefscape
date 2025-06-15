package frc.robot.superstructure.modifiers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.superstructure.PassiveModifier;
import frc.robot.superstructure.Superstructure.Subsystems;

public class ElevatorPrep implements PassiveModifier {
  /**
   * Modifies the behavior of the elevator's prefire. 
   */
  public ElevatorPrep() {}

  public void modify(Subsystems subsystems, Trigger trigger) {
    subsystems.elevator().setPrefireRequirement(trigger);
  }
 }
