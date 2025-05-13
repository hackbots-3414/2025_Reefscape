package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class ClimberConstants {
  protected static final int kLeftMotorID = 1;
  protected static final int kRightMotorID = 2;
  protected static final int kServoID = 7;
  protected static final int kEncoderID = 9;

  protected static final double kStowPosition = -0.25;
  protected static final double kClimbPosition = -0.110;
  protected static final double kClimbReadyTolerance = -0.001;

  protected static final double kFunnelOpenTime = 1.5;

  protected static final double kUpVolts = 12.0;
  protected static final double kDownVolts = -12.0;

  private static final double kSupplyCurrentLimit = 80.0;

  private static final double kForwardSoftLimit = 0.0;
  private static final double kReverseSoftLimit = -0.25;

  protected static final double kOpenServoPosition = 0.0;
  protected static final double kClosedServoPosition = 1.0;

  private static final double kEncoderOffset = -0.01318359;
  private static final SensorDirectionValue kEncoderDirection =
      SensorDirectionValue.CounterClockwise_Positive;

  protected static final CANcoderConfiguration kEncoderConfig = new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs()
          .withAbsoluteSensorDiscontinuityPoint(0.5)
          .withSensorDirection(kEncoderDirection)
          .withMagnetOffset(kEncoderOffset));

  protected static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.CounterClockwise_Positive))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(kSupplyCurrentLimit).withSupplyCurrentLimitEnable(true))

      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitThreshold(kForwardSoftLimit).withForwardSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(kReverseSoftLimit).withReverseSoftLimitEnable(true))

      .withFeedback(new FeedbackConfigs()
          .withFeedbackRemoteSensorID(ClimberConstants.kEncoderID)
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));
}

