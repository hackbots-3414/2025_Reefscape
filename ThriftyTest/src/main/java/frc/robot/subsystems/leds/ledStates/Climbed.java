package frc.robot.subsystems.leds.ledStates;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import frc.robot.subsystems.leds.LEDState;
import frc.robot.subsystems.leds.LEDs;

public class Climbed extends LEDState {
  public Climbed(LEDs leds) {
    super(leds);
  }

  protected ControlRequest leftFunnel(LEDs leds) {
    return leds.leftFunnel(RainbowAnimation::new)
      .withFrameRate(5)
      .withDirection(AnimationDirectionValue.Backward)
      .withBrightness(1);
  }

  protected ControlRequest rightFunnel(LEDs leds) {
    return leds.rightFunnel(RainbowAnimation::new)
      .withFrameRate(5)
      .withDirection(AnimationDirectionValue.Backward)
      .withBrightness(1);
  }

  protected ControlRequest leftElevator(LEDs leds) {
    return leds.leftElevator(RainbowAnimation::new)
      .withFrameRate(5)
      .withDirection(AnimationDirectionValue.Backward)
      .withBrightness(1);
  }

  protected ControlRequest rightElevator(LEDs leds) {
    return leds.rightElevator(RainbowAnimation::new)
      .withFrameRate(5)
      .withDirection(AnimationDirectionValue.Backward)
      .withBrightness(1);
  }
}
