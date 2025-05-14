package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOHardware implements ElevatorIO {
  private final TalonFX m_leftMotor;
  private final TalonFX m_rightMotor;

  private final CANrange m_CANrange;

  private final DynamicMotionMagicVoltage m_control;

  public ElevatorIOHardware() {
    m_rightMotor = new TalonFX(ElevatorConstants.kRightMotorID, "*");
    m_leftMotor = new TalonFX(ElevatorConstants.kLeftMotorID, "*");
    m_rightMotor.getConfigurator().apply(ElevatorConstants.kMotorConfig);
    m_leftMotor.getConfigurator().apply(ElevatorConstants.kMotorConfig);
    m_leftMotor
        .setControl(new Follower(ElevatorConstants.kRightMotorID, ElevatorConstants.kInvertLeft));
    m_rightMotor.setPosition(0.0);
    m_leftMotor.setPosition(0.0);

    m_CANrange = new CANrange(ElevatorConstants.kCANrangeID);
    m_CANrange.getConfigurator().apply(ElevatorConstants.kCANrangeConfig);

    m_control = new DynamicMotionMagicVoltage(
        0, // no position setpoint yet
        ElevatorConstants.kMaxSpeed,
        ElevatorConstants.kMaxAcceleration,
        ElevatorConstants.kMaxJerk);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
        m_leftMotor.getMotorVoltage(),
        m_leftMotor.getSupplyCurrent(),
        m_leftMotor.getDeviceTemp(),
        m_leftMotor.getVelocity(),
        m_leftMotor.getPosition()).isOK();
    inputs.rightMotorConnected = BaseStatusSignal.refreshAll(
        m_rightMotor.getMotorVoltage(),
        m_rightMotor.getSupplyCurrent(),
        m_rightMotor.getDeviceTemp(),
        m_rightMotor.getVelocity(),
        m_rightMotor.getPosition()).isOK();
    inputs.leftVoltage = m_leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightVoltage = m_rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftCurrent = m_leftMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightCurrent = m_rightMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leftTemp = m_leftMotor.getDeviceTemp().getValueAsDouble();
    inputs.rightTemp = m_rightMotor.getDeviceTemp().getValueAsDouble();
    inputs.leftVelocityRPS = m_leftMotor.getVelocity().getValueAsDouble();
    inputs.rightVelocityRPS = m_rightMotor.getVelocity().getValueAsDouble();
    inputs.leftPosition = m_leftMotor.getPosition().getValueAsDouble();
    inputs.rightPosition = m_rightMotor.getPosition().getValueAsDouble();
    inputs.position = inputs.rightPosition;

    inputs.zeroCANrangeConnected = BaseStatusSignal.refreshAll(
        m_CANrange.getIsDetected(),
        m_CANrange.getDistance(),
        m_CANrange.getSignalStrength()).isOK();
    inputs.zeroCANrangeDetected = m_CANrange.getIsDetected().getValue();
    inputs.zeroCANrangeDistance = m_CANrange.getDistance().getValueAsDouble();
    inputs.zeroCANrangeStrength = m_CANrange.getSignalStrength().getValueAsDouble();
  }

  public void setPosition(double position) {
    m_rightMotor.setControl(m_control.withPosition(position));
  }

  public void setVoltage(double voltage) {
    m_rightMotor.setVoltage(voltage);
  }

  public void enableLimits() {
    m_rightMotor.getConfigurator().apply(ElevatorConstants.kMotorConfig.SoftwareLimitSwitch);
    m_leftMotor.getConfigurator().apply(ElevatorConstants.kMotorConfig.SoftwareLimitSwitch);
  }

  public void disableLimits() {
    SoftwareLimitSwitchConfigs noLimits = new SoftwareLimitSwitchConfigs();
    m_rightMotor.getConfigurator().apply(noLimits);
    m_leftMotor.getConfigurator().apply(noLimits);
  }

  public void calibrateZero() {
    m_rightMotor.setPosition(0.0);
    m_leftMotor.setPosition(0.0);
  }
}
