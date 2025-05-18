package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public record LEDInputs(
    Trigger coralHeld,
    Trigger coralPresent,
    Trigger controllersOk,
    Trigger climbed) {
}
