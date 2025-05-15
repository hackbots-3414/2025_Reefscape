package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;

public final class ElevatorConstants {
  protected static final int kLeftMotorID = 51;
  protected static final int kRightMotorID = 52;
  protected static final int kCANrangeID = 53;

  protected static final boolean kInvertLeft = true;

  protected static final double kSupplyCurrentLimit = 100;

  protected static final double kRotorToSensorRatio = 5.2;
  protected static final double kSensorToMechanismRatio = 1;

  protected static final double kGearRatio = kRotorToSensorRatio * kSensorToMechanismRatio;

  private static final double kDrumRadius = Units.inchesToMeters(2.256 / 2);

  protected static final double kMetersToRotations = 1 / (kDrumRadius * 2 * Math.PI);

  protected static final double kInch = Units.inchesToMeters(1) * kMetersToRotations;

  protected static final double kReefOffset = 3 * kInch;

  protected static final double kForwardSoftLimit = 11.15;
  protected static final double kReverseSoftLimit = 0;

  protected static final double kZeroVoltage = -4.0;

  protected static final double kUnsafeRange = ElevatorState.L2.position() + 2 * kInch;

  protected static final double kTolerance = 0.06;

  protected static final double kMaxSpeed = 32;
  protected static final double kMaxAcceleration = 48;
  protected static final double kMaxJerk = 480;

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
          .withReverseSoftLimitEnable(false))

      .withSlot0(new Slot0Configs()
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKP(20)
          .withKI(0)
          .withKD(0)
          .withKS(0.125)
          .withKV(3.59 * (kDrumRadius * 2 * Math.PI))
          .withKA(0.05 * (kDrumRadius * 2 * Math.PI))
          .withKG(0.42))

      .withMotionMagic(new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(kMaxSpeed)
          .withMotionMagicAcceleration(kMaxAcceleration)
          .withMotionMagicJerk(kMaxJerk));

  protected static final CANrangeConfiguration kCANrangeConfig = new CANrangeConfiguration()
      .withFovParams(new FovParamsConfigs()
          .withFOVRangeX(6.75)
          .withFOVRangeY(6.75))
      .withProximityParams(new ProximityParamsConfigs()
          .withMinSignalStrengthForValidMeasurement(3500)
          .withProximityThreshold(0.13)
          .withProximityHysteresis(0));

  protected static final Time kRangeDebounceTime = Seconds.of(0.06);

  protected static final LinearSystem<N2, N1, N2> kPlant =
      LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(1), 1, 1, 1);
}

