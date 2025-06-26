package frc.robot.subsystems.coral;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.StatusSignalUtil;

public class CoralIOHardware implements CoralIO {
  private final TalonFX m_leftMotor;
  private final TalonFX m_rightMotor;

  private final CANrange m_frontCANrange;
  private final CANrange m_upperCANrange;
  private final CANrange m_innerCANrange;

  private double m_leftVoltage;
  private double m_rightVoltage;

  private final StatusSignal<Voltage> m_leftVoltageSignal;
  private final StatusSignal<Voltage> m_rightVoltageSignal;
  private final StatusSignal<Current> m_leftCurrentSignal;
  private final StatusSignal<Current> m_rightCurrentSignal;
  private final StatusSignal<Temperature> m_leftTempSignal;
  private final StatusSignal<Temperature> m_rightTempSignal;
  private final StatusSignal<AngularVelocity> m_leftVelocitySignal;
  private final StatusSignal<AngularVelocity> m_rightVelocitySignal;

  private final StatusSignal<Distance> m_frontDistanceSignal;
  private final StatusSignal<Distance> m_upperDistanceSignal;
  private final StatusSignal<Distance> m_innerDistanceSignal;
  private final StatusSignal<Boolean> m_frontDetectedSignal;
  private final StatusSignal<Boolean> m_upperDetectedSignal;
  private final StatusSignal<Boolean> m_innerDetectedSignal;
  private final StatusSignal<Double> m_frontStrengthSignal;
  private final StatusSignal<Double> m_upperStrengthSignal;
  private final StatusSignal<Double> m_innerStrengthSignal;

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

    m_leftVoltageSignal = m_leftMotor.getMotorVoltage();
    m_rightVoltageSignal = m_rightMotor.getMotorVoltage();
    m_leftCurrentSignal = m_leftMotor.getSupplyCurrent();
    m_rightCurrentSignal = m_rightMotor.getSupplyCurrent();
    m_leftTempSignal = m_leftMotor.getDeviceTemp();
    m_rightTempSignal = m_rightMotor.getDeviceTemp();
    m_leftVelocitySignal = m_leftMotor.getVelocity();
    m_rightVelocitySignal = m_rightMotor.getVelocity();

    m_frontDistanceSignal = m_frontCANrange.getDistance();
    m_upperDistanceSignal = m_upperCANrange.getDistance();
    m_innerDistanceSignal = m_innerCANrange.getDistance();
    m_frontStrengthSignal = m_frontCANrange.getSignalStrength();
    m_upperStrengthSignal = m_upperCANrange.getSignalStrength();
    m_innerStrengthSignal = m_innerCANrange.getSignalStrength();
    m_frontDetectedSignal = m_frontCANrange.getIsDetected();
    m_upperDetectedSignal = m_upperCANrange.getIsDetected();
    m_innerDetectedSignal = m_innerCANrange.getIsDetected();

    StatusSignalUtil.registerRioSignals(
        m_leftVoltageSignal,
        m_rightVoltageSignal,
        m_leftCurrentSignal,
        m_rightCurrentSignal,
        m_leftTempSignal,
        m_rightTempSignal,
        m_leftVelocitySignal,
        m_rightVelocitySignal,
        m_frontDetectedSignal,
        m_upperDetectedSignal,
        m_innerDetectedSignal,
        m_frontDistanceSignal,
        m_upperDistanceSignal,
        m_innerDistanceSignal,
        m_frontStrengthSignal,
        m_upperStrengthSignal,
        m_innerStrengthSignal);
  }

  public void updateInputs(CoralIOInputs inputs) {
    inputs.rightMotorConnected = BaseStatusSignal.isAllGood(
        m_rightVoltageSignal,
        m_rightCurrentSignal,
        m_rightTempSignal,
        m_rightVelocitySignal);
    inputs.leftMotorConnected = BaseStatusSignal.isAllGood(
        m_leftVoltageSignal,
        m_leftCurrentSignal,
        m_leftTempSignal,
        m_leftVelocitySignal);
    inputs.rightVoltage = m_rightVoltageSignal.getValueAsDouble();
    inputs.leftVoltage = m_leftVoltageSignal.getValueAsDouble();
    inputs.rightCurrent = m_rightCurrentSignal.getValueAsDouble();
    inputs.leftCurrent = m_leftCurrentSignal.getValueAsDouble();
    inputs.rightTemperature = m_rightTempSignal.getValueAsDouble();
    inputs.leftTemperature = m_leftTempSignal.getValueAsDouble();
    inputs.frontCANrangeConnected = BaseStatusSignal.isAllGood(
        m_frontDetectedSignal,
        m_frontDistanceSignal,
        m_frontStrengthSignal);
    inputs.upperCANrangeConnected = BaseStatusSignal.isAllGood(
        m_upperDetectedSignal,
        m_upperDistanceSignal,
        m_upperStrengthSignal);
    inputs.innerCANrangeConnected = BaseStatusSignal.isAllGood(
        m_innerDetectedSignal,
        m_innerDistanceSignal,
        m_innerStrengthSignal);
    inputs.frontDetected = m_frontDetectedSignal.getValue();
    inputs.upperDetected = m_upperDetectedSignal.getValue();
    inputs.innerDetected = m_innerDetectedSignal.getValue();
    inputs.frontDistance = m_frontDistanceSignal.getValueAsDouble();
    inputs.upperDistance = m_upperDistanceSignal.getValueAsDouble();
    inputs.innerDistance = m_innerDistanceSignal.getValueAsDouble();
    inputs.frontStrength = m_frontStrengthSignal.getValueAsDouble();
    inputs.upperStreingth = m_upperStrengthSignal.getValueAsDouble();
    inputs.innerStrength = m_innerStrengthSignal.getValueAsDouble();
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
