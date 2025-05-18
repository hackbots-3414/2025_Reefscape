package frc.robot.subsystems.leds;

import java.util.function.BiFunction;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.ledStates.BadController;
import frc.robot.subsystems.leds.ledStates.Climbed;
import frc.robot.subsystems.leds.ledStates.CoralHeld;
import frc.robot.subsystems.leds.ledStates.CoralPresent;
import frc.robot.subsystems.leds.ledStates.Enabled;
import frc.robot.utils.LoopTimer;

public class LEDs extends SubsystemBase {
  private final CANdle m_leftCANdle;
  private final CANdle m_rightCANdle;

  private final LoopTimer m_loopTimer;

  private final LEDState m_enabled;
  private final LEDState m_coralPresent;
  private final LEDState m_coralHeld;
  private final LEDState m_badController;
  private final LEDState m_climbed;

  private final LEDInputs m_inputs;

  public LEDs(LEDInputs inputs) {
    super();
    m_leftCANdle = new CANdle(LEDConstants.kLeftCANdleID);
    m_rightCANdle = new CANdle(LEDConstants.kRightCANdleID);

    m_inputs = inputs;

    m_enabled = new Enabled(this);
    m_coralPresent = new CoralPresent(this);
    m_coralHeld = new CoralHeld(this);
    m_badController = new BadController(this);
    m_climbed = new Climbed(this);

    m_loopTimer = new LoopTimer("LEDs");
  }

  @Override
  public void periodic() {
    m_loopTimer.reset();
    LEDState state = getActiveState();
    applyState(state);
    SmartDashboard.putString("LEDs/State", state.getClass().getSimpleName());
    m_loopTimer.log();
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

  private LEDState getActiveState() {
    if (!m_inputs.controllersOk().getAsBoolean()) {
      return m_badController;
    }
    if (m_inputs.climbed().getAsBoolean()) {
      return m_climbed;
    }
    if (m_inputs.coralHeld().getAsBoolean()) {
      return m_coralHeld;
    }
    if (m_inputs.coralPresent().getAsBoolean()) {
      return m_coralPresent;
    }
    return m_enabled;
  }
}
