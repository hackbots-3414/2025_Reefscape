package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.utils.StatusSignalUtil;

public class ClimberIOHardware implements ClimberIO {
  private final TalonFX m_leftMotor;
  private final TalonFX m_rightMotor;

  private final CANcoder m_CANcoder;

  private final Servo m_servo;

  private double m_voltage;

  private final StatusSignal<Voltage> m_leftVoltageSignal;
  private final StatusSignal<Voltage> m_rightVoltageSignal;
  private final StatusSignal<Current> m_leftCurrentSignal;
  private final StatusSignal<Current> m_rightCurrentSignal;
  private final StatusSignal<Temperature> m_leftTempSignal;
  private final StatusSignal<Temperature> m_rightTempSignal;
  private final StatusSignal<AngularVelocity> m_leftVelocitySignal;
  private final StatusSignal<AngularVelocity> m_rightVelocitySignal;

  private final StatusSignal<Angle> m_positionSignal;

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

    m_leftVoltageSignal = m_leftMotor.getMotorVoltage();
    m_rightVoltageSignal = m_rightMotor.getMotorVoltage();
    m_leftCurrentSignal = m_leftMotor.getSupplyCurrent();
    m_rightCurrentSignal = m_rightMotor.getSupplyCurrent();
    m_leftTempSignal = m_leftMotor.getDeviceTemp();
    m_rightTempSignal = m_rightMotor.getDeviceTemp();
    m_leftVelocitySignal = m_leftMotor.getVelocity();
    m_rightVelocitySignal = m_rightMotor.getVelocity();

    m_positionSignal = m_CANcoder.getPosition();

    StatusSignalUtil.registerRioSignals(
        m_leftVoltageSignal,
        m_rightVoltageSignal,
        m_leftCurrentSignal,
        m_rightCurrentSignal,
        m_leftTempSignal,
        m_rightTempSignal,
        m_leftVelocitySignal,
        m_rightVelocitySignal);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftConnected = BaseStatusSignal.isAllGood(
        m_leftVoltageSignal,
        m_leftCurrentSignal,
        m_leftTempSignal,
        m_leftVelocitySignal);
    inputs.rightConnected = BaseStatusSignal.isAllGood(
        m_rightVoltageSignal,
        m_rightCurrentSignal,
        m_rightTempSignal,
        m_rightVelocitySignal);
    inputs.leftVoltage = m_leftVoltageSignal.getValueAsDouble();
    inputs.rightVoltage = m_rightVoltageSignal.getValueAsDouble();
    inputs.leftCurrent = m_leftCurrentSignal.getValueAsDouble();
    inputs.rightCurrent = m_rightCurrentSignal.getValueAsDouble();
    inputs.leftTemp = m_leftTempSignal.getValueAsDouble();
    inputs.rightTemp = m_rightTempSignal.getValueAsDouble();
    inputs.leftVelocityRPS = m_leftVelocitySignal.getValueAsDouble();
    inputs.rightVelocityRPS = m_rightVelocitySignal.getValueAsDouble();
    inputs.encoderConnected = BaseStatusSignal.isAllGood(m_positionSignal);
    inputs.position = m_positionSignal.getValueAsDouble();
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
