package frc.robot.subsystems.leds;

import java.util.function.BiFunction;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PassiveSubsystem;
import frc.robot.subsystems.leds.ledStates.Enabled;

public class LEDs extends PassiveSubsystem {
  private final CANdle m_leftCANdle;
  private final CANdle m_rightCANdle;

  private final LEDState m_enabled;

  public LEDs() {
    super();
    m_leftCANdle = new CANdle(LEDConstants.kLeftCANdleID);
    m_rightCANdle = new CANdle(LEDConstants.kRightCANdleID);

    m_enabled = new Enabled(this);
  }

  protected void passive() {}

  @Override
  public void periodic() {
    LEDState state = getActiveState();
    applyState(state);
    SmartDashboard.putString("LEDs/State", state.getClass().getSimpleName());
    SmartDashboard.putString("LEDs/Left Control", m_leftCANdle.getAppliedControl().getName());
    SmartDashboard.putString("LEDs/Right Control", m_rightCANdle.getAppliedControl().getName());
  }

  private LEDState getActiveState() {
    return m_enabled;
  }

  private void applyState(LEDState state) {
    m_leftCANdle.setControl(state.leftFunnelControl);
    m_leftCANdle.setControl(state.leftElevatorControl);
    m_rightCANdle.setControl(state.rightFunnelControl);
    m_rightCANdle.setControl(state.rightElevatorControl);
  }

  public <T extends ControlRequest> T leftFunnel(BiFunction<Integer, Integer, T> control) {
    return control.apply(LEDConstants.kLeftFunnelStart, LEDConstants.kLeftFunnelEnd);
  }

  public <T extends ControlRequest> T rightFunnel(BiFunction<Integer, Integer, T> control) {
    return control.apply(LEDConstants.kRightFunnelStart, LEDConstants.kRightFunnelEnd);
  }

  public <T extends ControlRequest> T leftElevator(BiFunction<Integer, Integer, T> control) {
    return control.apply(LEDConstants.kLeftElevatorStart, LEDConstants.kLeftElevatorEnd);
  }

  public <T extends ControlRequest> T rightElevator(BiFunction<Integer, Integer, T> control) {
    return control.apply(LEDConstants.kRightElevatorStart, LEDConstants.kRightElevatorEnd);
  }
}
