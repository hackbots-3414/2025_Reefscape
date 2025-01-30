// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.StateSpaceConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class StateSpaceConstants {
        public static final double k_dt = 0.02;
        public static final double k_maxVoltage = 4.0;
    }

    public static final class AutonConstants {
        public static final int numWaypoints = 5;
    }

    public static final class ElevatorConstants {
        public static final int leftMotorID = 51;
        public static final int rightMotorID = 52;
        public static final int encoderPort = 53;

        public static final boolean invertRightMotor = true;

        public static final int forwardLimitChannelID = 0;
        public static final int reverseLimitChannelID = 1;

        public static final double forwardSoftLimit = 4;
        public static final double reverseSoftLimit = 0;

        public static final double supplyCurrentLimit = 40;

        public static final double rotorToSensorRatio = 5.2;
        public static final double sensorToMechanismRatio = 1;

        public static final InvertedValue motorInverted = InvertedValue.Clockwise_Positive;
        
        public static final double gearRatio = 5.2;
        
        public static final double carriageMass = Units.lbsToKilograms(14); // Mass of the elevator carriage
        public static final double drumRadius = Units.inchesToMeters(2.256 / 2); // Radius of the elevator drum
        public static final double metersPerRotation = (2 * Math.PI * drumRadius) / gearRatio;
        public static final double mpsPerRPM = metersPerRotation / 60.0;

        public static final double momentOfIntertia = 0.005715;

        public static final double tolerance = forwardSoftLimit * 0.01; // 1% tolerance

        private static final Vector<N2> stateSpaceStandardDeviations = VecBuilder.fill(12, 0.3);

        private static final Vector<N2> qelms = VecBuilder.fill(0.0001, 0.1);
        private static final Vector<N1> relms = VecBuilder.fill(1.0);

        public static final LinearSystem<N2, N1, N2> stateSpacePlant = LinearSystemId
                .createDCMotorSystem(TalonFXConstants.TalonFXDCMotor, momentOfIntertia, gearRatio);

        public static final StateSpaceConfig<N2, N1, N2> stateSpaceConfig = new StateSpaceConfig<N2, N1, N2>(
                stateSpacePlant,
                stateSpaceStandardDeviations,
                VecBuilder.fill(TalonFXConstants.positionStdDevs, TalonFXConstants.velocityStdDevs),
                qelms,
                relms,
                Nat.N2(),
                Nat.N2(),
                tolerance,
                "Elevator");

        public static final double absoluteSensorRange = 0.5;
        public static final SensorDirectionValue invertEncoder = SensorDirectionValue.CounterClockwise_Positive;
        public static final double encoderOffset = 0.324707;

        public static final double stow = 0;
        public static final double processor = 0.25;
        public static final double L1 = 0.5;
        public static final double L2 = 1.5;
        public static final double L3 = 2.5;
        public static final double L4 = 3.5;
        public static final double net = 4;

        public static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withAbsoluteSensorDiscontinuityPoint(absoluteSensorRange)
                        .withSensorDirection(invertEncoder)
                        .withMagnetOffset(encoderOffset));

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(motorInverted))

                .withFeedback(new FeedbackConfigs()
                        .withRotorToSensorRatio(rotorToSensorRatio)
                        .withSensorToMechanismRatio(sensorToMechanismRatio))

                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(supplyCurrentLimit))

                .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
                        .withForwardLimitEnable(true)
                        .withReverseLimitEnable(true)
                        .withReverseLimitAutosetPositionEnable(true)
                        .withReverseLimitAutosetPositionValue(0.0))

                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(forwardSoftLimit)
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(reverseSoftLimit)
                        .withForwardSoftLimitEnable(true));
    }

    public static final class PivotConstants {
        public static final int motorID = 61;
        public static final int encoderID = 62;
        public static final double encoderOffset = 0.324707;

        public static final double rotorToSensorRatio = 70.0 / 8.0;
        public static final double sensorToMechanismRatio = 32.0 / 14.0;

        public static final InvertedValue invertMotor = InvertedValue.Clockwise_Positive;
        public static final SensorDirectionValue invertEncoder = SensorDirectionValue.CounterClockwise_Positive;

        public static final double forwardSoftLimitThreshold = Math.PI / 2;
        public static final double reverseSoftLimitThreshold = 0;

        public static final double radiansAtMax = forwardSoftLimitThreshold;
        public static final double radiansAtZero = 0;

        public static final double absoluteSensorRange = 0.5;

        public static final double supplyCurrentLimit = 20;

        public static final double tolerance = forwardSoftLimitThreshold * 0.01; // 1% tolerance

        public static final double groundPickup = Math.PI / 2 * 4 / 5;
        public static final double processor = Math.PI / 5;
        public static final double reefPickup = Math.PI / 8;
        public static final double net = Math.PI / 2;
        public static final double stow = 0;

        private static final Vector<N2> stateSpaceStandardDeviation = VecBuilder.fill(0.1, 0.3);

        private static final Vector<N2> qelms = VecBuilder.fill(0.0001, 0.1);
        private static final Vector<N1> relms = VecBuilder.fill(4.0);

        public static final double momentOfIntertia = 0.14622;
        public static final double gearRatio = rotorToSensorRatio * sensorToMechanismRatio;

        public static final LinearSystem<N2, N1, N2> stateSpacePlant = LinearSystemId
                .createDCMotorSystem(TalonFXConstants.TalonFXDCMotor, momentOfIntertia, gearRatio);

        public static final StateSpaceConfig<N2, N1, N2> stateSpaceConfig = new StateSpaceConfig<N2, N1, N2>(
                stateSpacePlant,
                stateSpaceStandardDeviation,
                VecBuilder.fill(TalonFXConstants.positionStdDevs, TalonFXConstants.velocityStdDevs),
                qelms,
                relms,
                Nat.N2(),
                Nat.N2(),
                tolerance,
                "Pivot");

        public static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withAbsoluteSensorDiscontinuityPoint(absoluteSensorRange)
                        .withSensorDirection(invertEncoder)
                        .withMagnetOffset(encoderOffset));

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(invertMotor))

                .withFeedback(new FeedbackConfigs()
                        .withFeedbackRemoteSensorID(encoderID)
                        .withFeedbackRotorOffset(encoderOffset)
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                        .withRotorToSensorRatio(rotorToSensorRatio)
                        .withSensorToMechanismRatio(sensorToMechanismRatio))

                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(supplyCurrentLimit))

                .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
                        .withForwardLimitEnable(true)
                        .withReverseLimitEnable(true)
                        .withReverseLimitAutosetPositionEnable(true)
                        .withReverseLimitAutosetPositionValue(0.0))

                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(forwardSoftLimitThreshold)
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(reverseSoftLimitThreshold)
                        .withReverseSoftLimitEnable(true));

        public static final double armLength = 0.443;

    }

    public static final class TalonFXConstants {
        public final static double nominalVoltageVolts = 12.0; // DC Volts
        public final static double stallTorqueNewtonMeters = 4.69; // Nm
        public final static double stallCurrentAmps = 257.0; // Amps
        public final static double freeCurrentAmps = 1.5; // Amps
        public final static double freeSpeedRadPerSec = 6380.0 * 2.0 * Math.PI / 60.0; // RPM * 2pi / 60 = Rad per
                                                                                       // second

        public final static double positionStdDevs = 1.0 / 2048.0; // rotations
        public final static double velocityStdDevs = 2.0 / 2048.0; // rotations

        public final static DCMotor TalonFXDCMotor = new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters,
                stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, 1);
    }

}