package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class AlgaeConstants {
  public static final int kMotorID = 60;

  public static final double kIntakeVoltage = 12;
  public static final double kNetEjectVoltage = -3.0; // 3.0
  public static final double kProcessorEjectVoltage = -3.2;
  public static final double kHoldVoltage = 2.7;

  public static final double kTorqueCurrentThreshold = 75;
  public static final double kSupplyCurrentLimit = 25.0;

  public static final double kUpdateObjectPeriodSeconds = 0.200;

  public static final double kProcessorScoreTime = 2.0;
  public static final double kNetScoreTime = 0.4;

  public static final InvertedValue kInvertMotor = InvertedValue.Clockwise_Positive;

  public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(kInvertMotor))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(kSupplyCurrentLimit));
}

