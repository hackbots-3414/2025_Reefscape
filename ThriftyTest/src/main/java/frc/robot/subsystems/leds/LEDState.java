package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyControl;

public abstract class LEDState {
  protected abstract ControlRequest leftFunnel(LEDs leds);
  protected abstract ControlRequest rightFunnel(LEDs leds);
  protected abstract ControlRequest leftElevator(LEDs leds);
  protected abstract ControlRequest rightElevator(LEDs leds);

  public final ControlRequest leftFunnelControl;
  public final ControlRequest rightFunnelControl;
  public final ControlRequest leftElevatorControl;
  public final ControlRequest rightElevatorControl;

  public LEDState(LEDs leds) {
    leftFunnelControl = leftFunnel(leds);
    rightFunnelControl = rightFunnel(leds);
    leftElevatorControl = leftElevator(leds);
    rightElevatorControl = rightElevator(leds);
  }

  public LEDState() {
    leftFunnelControl = new EmptyControl();
    rightFunnelControl = new EmptyControl();
    leftElevatorControl = new EmptyControl();
    rightElevatorControl = new EmptyControl();
  }

  public static class None extends LEDState {
    protected ControlRequest leftFunnel(LEDs leds) {
      return new EmptyControl();
    }

    protected ControlRequest rightFunnel(LEDs leds) {
      return new EmptyControl();
    }

    protected ControlRequest leftElevator(LEDs leds) {
      return new EmptyControl();
    }

    protected ControlRequest rightElevator(LEDs leds) {
      return new EmptyControl();
    }
  }
}

