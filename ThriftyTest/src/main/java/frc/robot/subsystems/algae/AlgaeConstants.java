package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Time;

public final class AlgaeConstants {
  protected static final int kMotorID = 60;

  protected static final double kIntakeVoltage = 12;
  protected static final double kNetEjectVoltage = -3.0;
  protected static final double kProcessorEjectVoltage = -3.2;
  protected static final double kHoldVoltage = 6.0;

  protected static final double kTorqueCurrentThreshold = 75; // We should consider 40-55 range as well.
  protected static final Time kAlgaeDebounceTime = Seconds.of(0.8);

  protected static final double kSupplyCurrentLimit = 40.0;
  protected static final double kStatorCurrentLimit = 125.0;

  protected static final double kProcessorScoreTime = 2.0;
  protected static final double kNetScoreTime = 0.4;

  protected static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(kSupplyCurrentLimit)
          .withStatorCurrentLimit(kStatorCurrentLimit)
          .withStatorCurrentLimitEnable(true));
}

