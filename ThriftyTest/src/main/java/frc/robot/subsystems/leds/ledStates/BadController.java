package frc.robot.subsystems.leds.ledStates;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.LEDState;
import frc.robot.subsystems.leds.LEDs;

public class BadController extends LEDState {
  public BadController(LEDs leds) {
    super(leds);
  }

  protected ControlRequest leftFunnel(LEDs leds) {
    return leds.leftFunnel(StrobeAnimation::new)
      .withColor(new RGBWColor(Color.kRed))
      .withFrameRate(5);
  }

  protected ControlRequest rightFunnel(LEDs leds) {
    return leds.rightFunnel(StrobeAnimation::new)
      .withColor(new RGBWColor(Color.kRed))
      .withFrameRate(5);
  }
  protected ControlRequest leftElevator(LEDs leds) {
    return leds.leftElevator(StrobeAnimation::new)
      .withColor(new RGBWColor(Color.kRed))
      .withFrameRate(5);
  }
  protected ControlRequest rightElevator(LEDs leds) {
    return leds.leftFunnel(StrobeAnimation::new)
      .withColor(new RGBWColor(Color.kRed))
      .withFrameRate(5);
  }
}
