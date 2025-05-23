package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.StatusSignalUtil;

public class ElevatorIOHardware implements ElevatorIO {
  private final TalonFX m_leftMotor;
  private final TalonFX m_rightMotor;

  private final CANrange m_CANrange;

  private final DynamicMotionMagicVoltage m_control;

  private double m_reference = Double.NaN;

  private final StatusSignal<Voltage> m_leftVoltageSignal;
  private final StatusSignal<Voltage> m_rightVoltageSignal;
  private final StatusSignal<Current> m_leftCurrentSignal;
  private final StatusSignal<Current> m_rightCurrentSignal;
  private final StatusSignal<Temperature> m_leftTempSignal;
  private final StatusSignal<Temperature> m_rightTempSignal;
  private final StatusSignal<AngularVelocity> m_leftVelocitySignal;
  private final StatusSignal<AngularVelocity> m_rightVelocitySignal;
  private final StatusSignal<Angle> m_leftPositionSignal;
  private final StatusSignal<Angle> m_rightPositionSignal;

  private final StatusSignal<Distance> m_CANrangeDistanceSignal;
  private final StatusSignal<Boolean> m_CANrangeDetectedSignal;
  private final StatusSignal<Double> m_CANrangeStrengthSignal;

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

    m_leftVoltageSignal = m_leftMotor.getMotorVoltage();
    m_rightVoltageSignal = m_rightMotor.getMotorVoltage();
    m_leftCurrentSignal = m_leftMotor.getSupplyCurrent();
    m_rightCurrentSignal = m_rightMotor.getSupplyCurrent();
    m_leftTempSignal = m_leftMotor.getDeviceTemp();
    m_rightTempSignal = m_rightMotor.getDeviceTemp();
    m_leftVelocitySignal = m_leftMotor.getVelocity();
    m_rightVelocitySignal = m_rightMotor.getVelocity();
    m_leftPositionSignal = m_leftMotor.getPosition();
    m_rightPositionSignal = m_rightMotor.getPosition();

    m_CANrangeDetectedSignal = m_CANrange.getIsDetected();
    m_CANrangeDistanceSignal = m_CANrange.getDistance();
    m_CANrangeStrengthSignal = m_CANrange.getSignalStrength();

    StatusSignalUtil.registerCANivoreSignals(
        m_leftVoltageSignal,
        m_rightVoltageSignal,
        m_leftCurrentSignal,
        m_rightCurrentSignal,
        m_leftTempSignal,
        m_rightTempSignal,
        m_leftVelocitySignal,
        m_rightVelocitySignal,
        m_leftPositionSignal,
        m_rightPositionSignal);
    StatusSignalUtil.registerRioSignals(
      m_CANrangeDetectedSignal,
      m_CANrangeDistanceSignal,
      m_CANrangeStrengthSignal);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorConnected = BaseStatusSignal.isAllGood(
        m_leftVoltageSignal,
        m_leftCurrentSignal,
        m_leftTempSignal,
        m_leftVelocitySignal,
        m_leftPositionSignal);
    inputs.rightMotorConnected = BaseStatusSignal.isAllGood(
        m_rightVoltageSignal,
        m_rightCurrentSignal,
        m_rightTempSignal,
        m_rightVelocitySignal,
        m_rightPositionSignal);
    inputs.leftVoltage = m_leftVoltageSignal.getValueAsDouble();
    inputs.rightVoltage = m_rightVoltageSignal.getValueAsDouble();
    inputs.leftCurrent = m_leftCurrentSignal.getValueAsDouble();
    inputs.rightCurrent = m_rightCurrentSignal.getValueAsDouble();
    inputs.leftTemp = m_leftTempSignal.getValueAsDouble();
    inputs.rightTemp = m_rightTempSignal.getValueAsDouble();
    inputs.leftVelocityRPS = m_leftVelocitySignal.getValueAsDouble();
    inputs.rightVelocityRPS = m_rightVelocitySignal.getValueAsDouble();
    inputs.leftPosition = m_leftPositionSignal.getValueAsDouble();
    inputs.rightPosition = m_rightPositionSignal.getValueAsDouble();
    inputs.position = inputs.rightPosition;

    inputs.reference = m_reference;

    inputs.zeroCANrangeConnected = BaseStatusSignal.isAllGood(
        m_CANrangeDetectedSignal,
        m_CANrangeDistanceSignal,
        m_CANrangeStrengthSignal);
    inputs.zeroCANrangeDetected = m_CANrangeDetectedSignal.getValue();
    inputs.zeroCANrangeDistance = m_CANrangeDistanceSignal.getValueAsDouble();
    inputs.zeroCANrangeStrength = m_CANrangeStrengthSignal.getValueAsDouble();
  }

  public void setPosition(double reference) {
    m_rightMotor.setControl(m_control.withPosition(reference));
    m_reference = reference;
  }

  public void setVoltage(double voltage) {
    m_rightMotor.setVoltage(voltage);
  }

  public void enableLimits() {
    m_rightMotor.getConfigurator().apply(ElevatorConstants.kMotorConfig.SoftwareLimitSwitch);
    m_leftMotor.getConfigurator().apply(ElevatorConstants.kMotorConfig.SoftwareLimitSwitch);
  }

  public void disableLimits() {
    SoftwareLimitSwitchConfigs noLimits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(false)
        .withReverseSoftLimitEnable(false);
    m_rightMotor.getConfigurator().apply(noLimits);
    m_leftMotor.getConfigurator().apply(noLimits);
  }

  public void calibrateZero() {
    m_rightMotor.setPosition(0.0);
    m_leftMotor.setPosition(0.0);
  }
}
