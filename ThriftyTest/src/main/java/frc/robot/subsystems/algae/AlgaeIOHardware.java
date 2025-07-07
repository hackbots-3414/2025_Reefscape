package frc.robot.subsystems.algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.StatusSignalUtil;

public class AlgaeIOHardware implements AlgaeIO {
  private final TalonFX m_motor;

  private double m_voltage;

  private final StatusSignal<Voltage> m_voltageSignal;
  private final StatusSignal<Current> m_currentSignal;
  private final StatusSignal<Current> m_torqueSignal;
  private final StatusSignal<Temperature> m_tempSignal;
  private final StatusSignal<AngularVelocity> m_velocitySignal;

  public AlgaeIOHardware() {
    m_motor = new TalonFX(AlgaeConstants.kMotorID);
    m_motor.clearStickyFaults();
    m_motor.getConfigurator().apply(AlgaeConstants.kMotorConfig);

    m_voltageSignal = m_motor.getMotorVoltage();
    m_currentSignal = m_motor.getSupplyCurrent();
    m_torqueSignal = m_motor.getTorqueCurrent();
    m_tempSignal = m_motor.getDeviceTemp();
    m_velocitySignal = m_motor.getVelocity();

    StatusSignalUtil.registerRioSignals(
        m_voltageSignal,
        m_currentSignal,
        m_torqueSignal,
        m_tempSignal,
        m_velocitySignal);
  }

  public void updateInputs(AlgaeIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.isAllGood(
        m_voltageSignal,
        m_currentSignal,
        m_torqueSignal,
        m_tempSignal,
        m_velocitySignal);
    inputs.voltage = m_voltageSignal.getValueAsDouble();
    inputs.current = m_currentSignal.getValueAsDouble();
    inputs.torque = m_torqueSignal.getValueAsDouble();
    inputs.temperature = m_tempSignal.getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    if (voltage != m_voltage) {
      m_voltage = voltage;
      m_motor.setVoltage(voltage);
    }
  }
}
