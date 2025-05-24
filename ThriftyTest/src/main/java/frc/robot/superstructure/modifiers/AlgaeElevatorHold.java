// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.modifiers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.superstructure.PassiveModifier;
import frc.robot.superstructure.Superstructure.Subsystems;

public class AlgaeElevatorHold implements PassiveModifier {
    /** Tells the elevator not to move when holding algae and in reef zone */
    public AlgaeElevatorHold() {}

    public void modify(Subsystems subsystems, Trigger trigger) {
        subsystems.elevator().setStayRequirement(trigger);
    }
}
