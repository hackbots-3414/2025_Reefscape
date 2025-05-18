package frc.robot.subsystems.leds.ledStates;

import frc.robot.subsystems.leds.LEDs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.LEDState;

public class CoralPresent extends LEDState {
  public CoralPresent(LEDs leds) {
    super(leds);
  }

  protected ControlRequest leftFunnel(LEDs leds) {
    return leds.leftFunnel(SolidColor::new)
      .withColor(new RGBWColor(Color.kGray));
  }

  protected ControlRequest rightFunnel(LEDs leds) {
    return leds.rightFunnel(SolidColor::new)
      .withColor(new RGBWColor(Color.kGray));
  }

  protected ControlRequest leftElevator(LEDs leds) {
    return leds.leftElevator(SolidColor::new)
      .withColor(new RGBWColor(Color.kGray));
  }

  protected ControlRequest rightElevator(LEDs leds) {
    return leds.rightElevator(SolidColor::new)
      .withColor(new RGBWColor(Color.kGray));
  }
}
