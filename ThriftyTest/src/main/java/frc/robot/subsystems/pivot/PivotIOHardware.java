package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class PivotIOHardware implements PivotIO {
  private final TalonFX m_motor;

  private final MotionMagicVoltage m_control;

  public PivotIOHardware() {
    m_motor = new TalonFX(PivotConstants.kMotorID);
    m_motor.getConfigurator().apply(PivotConstants.kMotorConfig);
    m_motor.setPosition(PivotConstants.kRotorOffset);

    m_control = new MotionMagicVoltage(0.0);
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.refreshAll(
        m_motor.getMotorVoltage(),
        m_motor.getSupplyCurrent(),
        m_motor.getDeviceTemp(),
        m_motor.getVelocity(),
        m_motor.getPosition()).isOK();
    inputs.voltage = m_motor.getMotorVoltage().getValueAsDouble();
    inputs.current = m_motor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatue = m_motor.getDeviceTemp().getValueAsDouble();
    inputs.velocityRPS = m_motor.getVelocity().getValueAsDouble();
    inputs.position = m_motor.getPosition().getValueAsDouble();
  }

  public void setPosition(double position, boolean holdingAlgae) {
    m_motor.setControl(m_control.withPosition(position).withSlot(holdingAlgae ? 1 : 0));
  }
}
