package frc.robot.subsystems.leds.ledStates;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.LEDState;
import frc.robot.subsystems.leds.LEDs;

public class Enabled extends LEDState {
  public Enabled(LEDs leds) {
    super(leds);
  }

  protected ControlRequest leftFunnel(LEDs leds) {
    return leds.leftFunnel(LarsonAnimation::new)
      .withColor(new RGBWColor(Color.kPurple))
      .withSize(7)
      .withBounceMode(LarsonBounceValue.Front)
      .withFrameRate(2);
  }

  protected ControlRequest rightFunnel(LEDs leds) {
    return leds.rightFunnel(LarsonAnimation::new)
      .withColor(new RGBWColor(Color.kPurple))
      .withSize(7)
      .withBounceMode(LarsonBounceValue.Front)
      .withFrameRate(2);
  }

  protected ControlRequest leftElevator(LEDs leds) {
    return leds.leftElevator(SingleFadeAnimation::new)
      .withColor(new RGBWColor(Color.kPurple));
  }

  protected ControlRequest rightElevator(LEDs leds) {
    return leds.rightElevator(SingleFadeAnimation::new)
      .withColor(new RGBWColor(Color.kPurple));
  }
}
