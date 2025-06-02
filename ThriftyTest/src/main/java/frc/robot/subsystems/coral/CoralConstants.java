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
  protected static final int kLeftMotorID = 55;
  protected static final int kRightMotorID = 56;
  protected static final int kFrontCANrangeID = 59;
  protected static final int kUpperCANrangeID = 58;
  protected static final int kInnerCANrangeID = 54;

  protected static final double kIntakeVoltage = 2.4;
  protected static final double kEjectVoltage = -6;

  protected static final double kL1EjectVoltage = 2.5;
  protected static final double kL2EjectVoltage = 4.0; // 5.1
  protected static final double kL3EjectVoltage = 4.0; // 5.1
  protected static final double kL4EjectVoltage = 5.5;

  protected static final double kL1LeftEjectVoltage = 2;
  protected static final double kL1RightEjectVoltage = 4;

  protected static final double kSupplyCurrentLimit = 20.0;

  protected static final InvertedValue kInvertRight = InvertedValue.Clockwise_Positive;

  protected static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.CounterClockwise_Positive))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(kSupplyCurrentLimit));

  protected static final CANrangeConfiguration kFrontCANrangeConfig = new CANrangeConfiguration()
      .withFovParams(new FovParamsConfigs()
          .withFOVRangeX(6.5)
          .withFOVRangeY(6.5))
      .withProximityParams(new ProximityParamsConfigs()
          .withMinSignalStrengthForValidMeasurement(15015)
          .withProximityThreshold(0.1))
      .withToFParams(new ToFParamsConfigs()
          .withUpdateMode(UpdateModeValue.ShortRange100Hz));


  protected static final CANrangeConfiguration kUpperCANrangeConfig = new CANrangeConfiguration()
      .withFovParams(new FovParamsConfigs()
          .withFOVRangeX(6.5)
          .withFOVRangeY(15))
      .withProximityParams(new ProximityParamsConfigs()
          .withMinSignalStrengthForValidMeasurement(2500)
          .withProximityThreshold(0.65))
      .withToFParams(new ToFParamsConfigs()
          .withUpdateMode(UpdateModeValue.ShortRange100Hz));

  protected static final CANrangeConfiguration kInnerCANrangeConfig = new CANrangeConfiguration()
      .withFovParams(new FovParamsConfigs()
          .withFOVRangeX(27)
          .withFOVRangeY(27))
      .withProximityParams(new ProximityParamsConfigs()
          .withMinSignalStrengthForValidMeasurement(1500)
          .withProximityHysteresis(0)
          .withProximityThreshold(0.06))
      .withToFParams(new ToFParamsConfigs()
          .withUpdateMode(UpdateModeValue.ShortRange100Hz));
}
