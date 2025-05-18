package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.StatusSignalUtil;

public class PivotIOHardware implements PivotIO {
  private final TalonFX m_motor;

  private final MotionMagicVoltage m_control;

  private final StatusSignal<Voltage> m_voltageSignal;
  private final StatusSignal<Current> m_currentSignal;
  private final StatusSignal<Temperature> m_tempSignal;
  private final StatusSignal<AngularVelocity> m_velocitySignal;
  private final StatusSignal<Angle> m_positionSignal;

  public PivotIOHardware() {
    m_motor = new TalonFX(PivotConstants.kMotorID);
    m_motor.getConfigurator().apply(PivotConstants.kMotorConfig);
    m_motor.setPosition(PivotConstants.kRotorOffset);

    m_voltageSignal = m_motor.getMotorVoltage();
    m_currentSignal = m_motor.getSupplyCurrent();
    m_tempSignal = m_motor.getDeviceTemp();
    m_velocitySignal = m_motor.getVelocity();
    m_positionSignal = m_motor.getPosition();

    m_control = new MotionMagicVoltage(0.0);

    StatusSignalUtil.registerRioSignals(
        m_voltageSignal,
        m_currentSignal,
        m_tempSignal,
        m_velocitySignal,
        m_positionSignal);
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.isAllGood(
        m_voltageSignal,
        m_currentSignal,
        m_tempSignal,
        m_velocitySignal,
        m_positionSignal);
    inputs.voltage = m_voltageSignal.getValueAsDouble();
    inputs.current = m_currentSignal.getValueAsDouble();
    inputs.temperatue = m_tempSignal.getValueAsDouble();
    inputs.velocityRPS = m_velocitySignal.getValueAsDouble();
    inputs.position = m_positionSignal.getValueAsDouble();
  }

  public void setPosition(double position, boolean holdingAlgae) {
    m_motor.setControl(m_control.withPosition(position).withSlot(holdingAlgae ? 1 : 0));
  }
}
