package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.KrakenX60Constants;

public final class ElevatorConstants {
  protected static final boolean enable = true;

  protected static final boolean invertLeftMotorFollower = true;

  protected static final double supplyCurrentLimit = 100;
  protected static final double k_zeroCurrentThreshold = 23.5;

  protected static final double rotorToSensorRatio = 5.2;
  protected static final double sensorToMechanismRatio = 1;

  protected static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;

  protected static final double gearRatio = rotorToSensorRatio * sensorToMechanismRatio;

  protected static final double stage1Mass = Units.lbsToKilograms(5.402);
  protected static final double stage2Mass = Units.lbsToKilograms(4.819);
  protected static final double carriageMass = Units.lbsToKilograms(3.084);
  protected static final double coralMechanismMass = Units.lbsToKilograms(8.173); // includes coral
  protected static final double algaeMechanismMass = Units.lbsToKilograms(8.359);

  // Mass of the elevator carriage
  protected static final double netMass =
      stage1Mass + stage2Mass + carriageMass + coralMechanismMass + algaeMechanismMass;

  // Radius of the elevator drum
  // approx. 0.02865
  protected static final double drumRadius = Units.inchesToMeters(2.256 / 2);

  protected static final LinearSystem<N2, N1, N2> stateSpacePlant = LinearSystemId
      .createElevatorSystem(KrakenX60Constants.KrakenX60Motor, netMass, drumRadius,
          gearRatio);

  protected static final double absoluteSensorRange = 0.5;
  protected static final SensorDirectionValue invertEncoder =
      SensorDirectionValue.CounterClockwise_Positive;
  protected static final double encoderOffset = 0.291015625; // 0.490234375

  protected static final double metersToRotations = 1 / (drumRadius * 2 * Math.PI);
  // approx 7.96

  protected static final boolean enableCANRange = true;

  protected static final double rangeDistanceGain = 64; // how much higher, per unit of range

  protected static final double inch = Units.inchesToMeters(1) * metersToRotations;

  protected static final double forwardSoftLimit = 11.15;
  protected static final double reverseSoftLimit = 0;

  protected static final double unsafeRange = ElevatorState.L2.position() + 2 * inch;

  protected static final double tolerance = 0.06;

  protected static final double k_maxCanCompensation = 2 * inch;

  protected static final double manualUpSpeed = 0.2;
  protected static final double manualDownSpeed = -0.3;

  protected static final double maxSpeedUp = 32; // 16
  protected static final double maxAccelerationUp = 48; // 48
  protected static final double maxJerkUp = 480; // 480

  protected static final double maxSpeedDown = 10; // 10
  protected static final double maxAccelerationDown = 30; // 30
  protected static final double maxJerkDown = 300; // 300

  protected static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs()
          .withAbsoluteSensorDiscontinuityPoint(absoluteSensorRange)
          .withSensorDirection(invertEncoder)
          .withMagnetOffset(encoderOffset));

  protected static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(motorInverted))

      .withFeedback(new FeedbackConfigs()
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
          .withSensorToMechanismRatio(gearRatio))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(supplyCurrentLimit))

      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitThreshold(forwardSoftLimit)
          .withForwardSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(reverseSoftLimit)
          .withReverseSoftLimitEnable(false))

      .withSlot0(new Slot0Configs()
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKP(20)
          .withKI(0)
          .withKD(0)
          .withKS(0.125)
          .withKV(3.59 * (drumRadius * 2 * Math.PI))
          .withKA(0.05 * (drumRadius * 2 * Math.PI))
          .withKG(0.42))

      .withSlot1(new Slot1Configs()
          .withGravityType(GravityTypeValue.Elevator_Static)
          .withKP(7)
          .withKI(0)
          .withKD(0)
          .withKS(0)
          .withKV(3.59 * (drumRadius * 2 * Math.PI))
          .withKA(0.05 * (drumRadius * 2 * Math.PI))
          .withKG(0.42))

      .withMotionMagic(new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(maxSpeedUp)
          .withMotionMagicAcceleration(maxAccelerationUp)
          .withMotionMagicJerk(maxJerkUp));

  protected static final CANrangeConfiguration kCANrangeConfig = new CANrangeConfiguration()
      .withFovParams(new FovParamsConfigs()
          .withFOVRangeX(6.75)
          .withFOVRangeY(6.75))
      .withProximityParams(new ProximityParamsConfigs()
          .withMinSignalStrengthForValidMeasurement(3500)
          .withProximityThreshold(0.13)
          .withProximityHysteresis(0));

  protected static final Time kRangeDebounceTime = Seconds.of(0.06);
}

