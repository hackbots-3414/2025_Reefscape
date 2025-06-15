package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.superstructure.Superstructure.Subsystems;

public interface PassiveModifier {
  void modify(Subsystems subsystems, Trigger trigger);
}
