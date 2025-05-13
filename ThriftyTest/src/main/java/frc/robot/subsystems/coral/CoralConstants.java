package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class CoralConstants {
  public static final int kLeftMotorID = 55;
  public static final int kRightMotorID = 56;
  public static final int kFrontCANrangeID = 59;
  public static final int kUpperCANrangeID = 58;
  public static final int kInnerCANrangeID = 54;

  protected static final double kIntakeVoltage = 2.4;
  protected static final double retractVoltage = -3.5;
  protected static final double kEjectVoltage = -5;

  protected static final double l1EjectVoltage = 2.5;
  protected static final double kL2EjectVoltage = 4.0; // 5.1
  protected static final double kL3EjectVoltage = 4.0; // 5.1
  protected static final double kL4EjectVoltage = 5.5;

  protected static final double reverseEjectVoltage = -6;
  protected static final double fastEjectVoltage = -10;

  protected static final double kL1LeftEjectVoltage = 2;
  protected static final double kL1RightEjectVoltage = 4;

  protected static final boolean rightMotorInvert = true;

  protected static final double supplyCurrentLimit = 20.0;

  protected static final double IRThreshold = 0.51;

  protected static final boolean enableCANRange = true;

  protected static final InvertedValue kInvertRight = InvertedValue.Clockwise_Positive;

  protected static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.CounterClockwise_Positive))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(supplyCurrentLimit));

  protected static final CANrangeConfiguration kFrontRangeConfig = new CANrangeConfiguration()
      .withFovParams(new FovParamsConfigs()
          .withFOVRangeX(6.5)
          .withFOVRangeY(6.5))
      .withProximityParams(new ProximityParamsConfigs()
          .withMinSignalStrengthForValidMeasurement(15015)
          .withProximityThreshold(0.1))
      .withToFParams(new ToFParamsConfigs()
          .withUpdateMode(UpdateModeValue.ShortRange100Hz));


  protected static final CANrangeConfiguration kUpperRangeConfig = new CANrangeConfiguration()
      .withFovParams(new FovParamsConfigs()
          .withFOVRangeX(6.5)
          .withFOVRangeY(15))
      .withProximityParams(new ProximityParamsConfigs()
          .withMinSignalStrengthForValidMeasurement(2500)
          .withProximityThreshold(0.65))
      .withToFParams(new ToFParamsConfigs()
          .withUpdateMode(UpdateModeValue.ShortRange100Hz));

  protected static final CANrangeConfiguration kInnerRangeConfig = new CANrangeConfiguration()
      .withFovParams(new FovParamsConfigs()
          .withFOVRangeX(27)
          .withFOVRangeY(27))
      .withProximityParams(new ProximityParamsConfigs()
          .withMinSignalStrengthForValidMeasurement(1500)
          .withProximityHysteresis(0)
          .withProximityThreshold(0.06))
      .withToFParams(new ToFParamsConfigs()
          .withUpdateMode(UpdateModeValue.ShortRange100Hz));

  protected static double intakeTimeout = 1;
}
