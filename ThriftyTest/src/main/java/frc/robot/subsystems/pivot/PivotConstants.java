package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants.TalonFXConstants;

public final class PivotConstants {
  public static final int kMotorID = 57;

  public static final double encoderOffset = 0.665283203125;

  public static final double kRotorOffset = 0.344;

  public static final double rotorToSensorRatio = 64.0 / 14.0;
  public static final double sensorToMechanismRatio = 32.0 / 14.0;

  public static final InvertedValue invertMotor = InvertedValue.CounterClockwise_Positive;
  public static final SensorDirectionValue invertEncoder =
      SensorDirectionValue.Clockwise_Positive;

  public static final double forwardSoftLimitThreshold = 0.359;
  public static final double reverseSoftLimitThreshold = 0.0;

  public static final double kRadiansAtMax = forwardSoftLimitThreshold;
  public static final double kRadiansAtZero = 0;

  public static final double absoluteSensorRange = 0.5;

  public static final double supplyCurrentLimit = 40;

  public static final double kTolerance = 0.03;

  public static final double manualUpSpeed = 0.1;
  public static final double manualDownSpeed = -0.1;

  public static final double momentOfIntertia = 0.14622;
  public static final double kGearRatio = rotorToSensorRatio * sensorToMechanismRatio;

  public static final LinearSystem<N2, N1, N2> kPlant = LinearSystemId
      .createSingleJointedArmSystem(TalonFXConstants.TalonFXDCMotor, momentOfIntertia, kGearRatio);

  public static final double maxSpeed = 1.5; // cancoder rotations per second
  public static final double accelerationMultiplier = 2;

  public static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs()
          .withAbsoluteSensorDiscontinuityPoint(absoluteSensorRange)
          .withSensorDirection(invertEncoder)
          .withMagnetOffset(encoderOffset));

  public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(invertMotor))

      // .withFeedback(new FeedbackConfigs()
      // .withFeedbackRemoteSensorID(IDConstants.pivotEncoder)
      // .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
      // .withRotorToSensorRatio(rotorToSensorRatio)
      // .withSensorToMechanismRatio(sensorToMechanismRatio))
      .withFeedback(new FeedbackConfigs()
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
          .withSensorToMechanismRatio(kGearRatio))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(supplyCurrentLimit))

      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitThreshold(forwardSoftLimitThreshold)
          .withForwardSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(reverseSoftLimitThreshold)
          .withReverseSoftLimitEnable(true))

      .withSlot0(new Slot0Configs()
          .withGravityType(GravityTypeValue.Arm_Cosine)
          .withKP(25)
          .withKI(0)
          .withKD(0)
          .withKS(0)
          .withKV(1.3)
          .withKA(0.12)
          .withKG(0.625))
      .withSlot1(new Slot1Configs()
          .withGravityType(GravityTypeValue.Arm_Cosine)
          .withKP(30)
          .withKI(0)
          .withKD(0)
          .withKS(0)
          .withKV(1.3)
          .withKA(0.12)
          .withKG(0.85))

      .withMotionMagic(new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(maxSpeed)
          .withMotionMagicAcceleration(maxSpeed * accelerationMultiplier)
          .withMotionMagicJerk(maxSpeed * accelerationMultiplier * 10));

  public static final double kArmLength = 0.443;
}
