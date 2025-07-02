package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class AlgaeConstants {
  protected static final int kMotorID = 60;

  protected static final double kIntakeVoltage = 12;
  protected static final double kNetEjectVoltage = -3.0;
  protected static final double kProcessorEjectVoltage = -3.2;
  protected static final double kHoldVoltage = 6.5;

  protected static final double kTorqueCurrentThreshold = 95;
  protected static final double kSupplyCurrentLimit = 55.0;

  protected static final double kProcessorScoreTime = 2.0;
  protected static final double kNetScoreTime = 0.4;

  protected static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(kSupplyCurrentLimit));
}

