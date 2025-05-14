package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;

public class ClimberIOHardware implements ClimberIO {
  private final TalonFX m_leftMotor;
  private final TalonFX m_rightMotor;

  private final CANcoder m_CANcoder;

  private final Servo m_servo;

  private double m_voltage;

  public ClimberIOHardware() {
    m_leftMotor = new TalonFX(ClimberConstants.kLeftMotorID);
    m_rightMotor = new TalonFX(ClimberConstants.kRightMotorID);
    m_leftMotor.clearStickyFaults();
    m_rightMotor.clearStickyFaults();
    m_leftMotor.getConfigurator().apply(ClimberConstants.kMotorConfig);
    m_rightMotor.getConfigurator().apply(ClimberConstants.kMotorConfig);
    m_rightMotor.setControl(new Follower(ClimberConstants.kLeftMotorID, true));

    m_CANcoder = new CANcoder(ClimberConstants.kEncoderID);
    m_CANcoder.getConfigurator().apply(ClimberConstants.kEncoderConfig);

    m_servo = new Servo(ClimberConstants.kServoID);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftConnected = BaseStatusSignal.refreshAll(
        m_leftMotor.getMotorVoltage(),
        m_leftMotor.getSupplyCurrent(),
        m_leftMotor.getDeviceTemp(),
        m_leftMotor.getVelocity()).isOK();
    inputs.rightConnected = BaseStatusSignal.refreshAll(
        m_rightMotor.getMotorVoltage(),
        m_rightMotor.getSupplyCurrent(),
        m_rightMotor.getDeviceTemp(),
        m_rightMotor.getVelocity()).isOK();
    inputs.leftVoltage = m_leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightVoltage = m_rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftCurrent = m_leftMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightCurrent = m_rightMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leftTemp = m_leftMotor.getDeviceTemp().getValueAsDouble();
    inputs.rightTemp = m_rightMotor.getDeviceTemp().getValueAsDouble();
    inputs.leftVelocityRPS = m_leftMotor.getVelocity().getValueAsDouble();
    inputs.rightVelocityRPS = m_rightMotor.getVelocity().getValueAsDouble();
    inputs.position = m_CANcoder.getPosition().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    if (m_voltage != voltage) {
      m_voltage = voltage;
      m_leftMotor.setVoltage(voltage);
    }
  }

  public void setServo(double position) {
    m_servo.setPosition(position);
  }
}
