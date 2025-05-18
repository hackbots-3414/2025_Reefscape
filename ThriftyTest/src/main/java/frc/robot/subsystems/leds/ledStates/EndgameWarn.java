package frc.robot.subsystems.leds.ledStates;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.LEDState;
import frc.robot.subsystems.leds.LEDs;

public class EndgameWarn extends LEDState {
  public EndgameWarn(LEDs leds) {
    super(leds);
  }

  protected ControlRequest leftFunnel(LEDs leds) {
    return leds.leftFunnel(StrobeAnimation::new)
      .withFrameRate(2)
      .withColor(new RGBWColor(Color.kYellow));
  }

  protected ControlRequest rightFunnel(LEDs leds) {
    return leds.rightFunnel(StrobeAnimation::new)
      .withFrameRate(2)
      .withColor(new RGBWColor(Color.kYellow));
  }

  protected ControlRequest leftElevator(LEDs leds) {
    return leds.leftElevator(StrobeAnimation::new)
      .withFrameRate(2)
      .withColor(new RGBWColor(Color.kYellow));
  }

  protected ControlRequest rightElevator(LEDs leds) {
    return leds.rightElevator(StrobeAnimation::new)
      .withFrameRate(2)
      .withColor(new RGBWColor(Color.kYellow));
  }
}
