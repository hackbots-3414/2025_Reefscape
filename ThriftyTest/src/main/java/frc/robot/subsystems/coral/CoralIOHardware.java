package frc.robot.subsystems.coral;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public class CoralIOHardware implements CoralIO {
  private final TalonFX m_leftMotor;
  private final TalonFX m_rightMotor;

  private final CANrange m_frontCANrange;
  private final CANrange m_upperCANrange;
  private final CANrange m_innerCANrange;

  private double m_leftVoltage;
  private double m_rightVoltage;

  public CoralIOHardware() {
    m_leftMotor = new TalonFX(CoralConstants.kLeftMotorID);
    m_rightMotor = new TalonFX(CoralConstants.kRightMotorID);

    m_frontCANrange = new CANrange(CoralConstants.kFrontCANrangeID);
    m_upperCANrange = new CANrange(CoralConstants.kUpperCANrangeID);
    m_innerCANrange = new CANrange(CoralConstants.kInnerCANrangeID);

    m_leftMotor.getConfigurator().apply(CoralConstants.kMotorConfig);
    m_rightMotor.getConfigurator().apply(CoralConstants.kMotorConfig.withMotorOutput(
        CoralConstants.kMotorConfig.MotorOutput.withInverted(CoralConstants.kInvertRight)));

    m_frontCANrange.getConfigurator().apply(CoralConstants.kFrontCANrangeConfig);
    m_upperCANrange.getConfigurator().apply(CoralConstants.kUpperCANrangeConfig);
    m_innerCANrange.getConfigurator().apply(CoralConstants.kInnerCANrangeConfig);
  }

  public void updateInputs(CoralIOInputs inputs) {
    inputs.rightMotorConnected = BaseStatusSignal.refreshAll(
        m_rightMotor.getMotorVoltage(),
        m_rightMotor.getSupplyCurrent(),
        m_rightMotor.getDeviceTemp(),
        m_rightMotor.getVelocity()).isOK();
    inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
        m_leftMotor.getMotorVoltage(),
        m_leftMotor.getSupplyCurrent(),
        m_leftMotor.getDeviceTemp(),
        m_leftMotor.getVelocity()).isOK();
    inputs.rightVoltage = m_rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftVoltage = m_leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightCurrent = m_rightMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leftCurrent = m_leftMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightTemperature = m_rightMotor.getDeviceTemp().getValueAsDouble();
    inputs.leftTemperature = m_leftMotor.getDeviceTemp().getValueAsDouble();
    inputs.frontCANrangeConnected = BaseStatusSignal.refreshAll(
        m_frontCANrange.getIsDetected(),
        m_frontCANrange.getDistance(),
        m_frontCANrange.getSignalStrength()).isOK();
    inputs.upperCANrangeConnected = BaseStatusSignal.refreshAll(
        m_upperCANrange.getIsDetected(),
        m_upperCANrange.getDistance(),
        m_upperCANrange.getSignalStrength()).isOK();
    inputs.innerCANrangeConnected = BaseStatusSignal.refreshAll(
        m_innerCANrange.getIsDetected(),
        m_innerCANrange.getDistance(),
        m_upperCANrange.getSignalStrength()).isOK();
    inputs.frontDetected = m_frontCANrange.getIsDetected().getValue();
    inputs.upperDetected = m_upperCANrange.getIsDetected().getValue();
    inputs.innerDetected = m_innerCANrange.getIsDetected().getValue();
  }

  public void setLeftVoltage(double voltage) {
    if (m_leftVoltage != voltage) {
      m_leftVoltage = voltage;
      m_leftMotor.setVoltage(voltage);
    }
  }

  public void setRightVoltage(double voltage) {
    if (m_rightVoltage != voltage) {
      m_rightVoltage = voltage;
      m_rightMotor.setVoltage(voltage);
    }
  }
}
