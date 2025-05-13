package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants.TalonFXConstants;

public final class PivotConstants {
  protected static final int kMotorID = 57;

  protected static final double kRotorOffset = 0.344;

  protected static final double kRotorToSensorRatio = 64.0 / 14.0;
  protected static final double kSensorToMechanismRatio = 32.0 / 14.0;

  protected static final double kForwardSoftLimit = 0.359;
  protected static final double kReverseSoftLimit = 0.0;

  protected static final double kSupplyCurrentLimit = 40;

  protected static final double kTolerance = 0.03;

  protected static final double kMomentOfIntertia = 0.14622;
  protected static final double kGearRatio = kRotorToSensorRatio * kSensorToMechanismRatio;

  protected static final LinearSystem<N2, N1, N2> kPlant = LinearSystemId
      .createSingleJointedArmSystem(TalonFXConstants.TalonFXDCMotor, kMomentOfIntertia, kGearRatio);

  protected static final double kMaxSpeed = 1.5; // cancoder rotations per secon
  protected static final double kMaxAcceleration = 3.0;
  protected static final double kMaxJerk = 30.0;

  protected static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.CounterClockwise_Positive))

      .withFeedback(new FeedbackConfigs()
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
          .withSensorToMechanismRatio(kGearRatio))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(kSupplyCurrentLimit))

      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitThreshold(kForwardSoftLimit)
          .withForwardSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(kReverseSoftLimit)
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
          .withMotionMagicCruiseVelocity(kMaxSpeed)
          .withMotionMagicAcceleration(kMaxAcceleration)
          .withMotionMagicJerk(kMaxJerk));

  protected static final double kArmLength = 0.443;
}
