package frc.robot.subsystems.algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

public class AlgaeIOHardware implements AlgaeIO {
  private final TalonFX m_motor;

  private double m_voltage;

  public AlgaeIOHardware() {
    m_motor = new TalonFX(AlgaeConstants.kMotorID);
  }

  public void updateInputs(AlgaeIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.refreshAll(
        m_motor.getMotorVoltage(),
        m_motor.getSupplyCurrent(),
        m_motor.getTorqueCurrent(),
        m_motor.getDeviceTemp(),
        m_motor.getVelocity()).isOK();
    inputs.current = m_motor.getSupplyCurrent().getValueAsDouble();
    inputs.torque = m_motor.getTorqueCurrent().getValueAsDouble();
    inputs.temperature = m_motor.getDeviceTemp().getValueAsDouble();
    inputs.voltage = m_voltage;
    inputs.velocityRPS = m_motor.getVelocity().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    if (voltage != m_voltage) {
      m_voltage = voltage;
      m_motor.setVoltage(voltage);
    }
  }
}
