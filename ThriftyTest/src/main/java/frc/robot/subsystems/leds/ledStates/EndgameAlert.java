package frc.robot.subsystems.leds.ledStates;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.LEDState;
import frc.robot.subsystems.leds.LEDs;

public class EndgameAlert extends LEDState {
  public EndgameAlert(LEDs leds) {
    super(leds);
  }

  protected ControlRequest leftFunnel(LEDs leds) {
    return leds.leftFunnel(SolidColor::new)
      .withColor(new RGBWColor(Color.kYellow));
  }

  protected ControlRequest rightFunnel(LEDs leds) {
    return leds.rightFunnel(SolidColor::new)
      .withColor(new RGBWColor(Color.kYellow));
  }

  protected ControlRequest leftElevator(LEDs leds) {
    return leds.leftElevator(SolidColor::new)
      .withColor(new RGBWColor(Color.kYellow));
  }

  protected ControlRequest rightElevator(LEDs leds) {
    return leds.rightElevator(SolidColor::new)
      .withColor(new RGBWColor(Color.kYellow));
  }
}
