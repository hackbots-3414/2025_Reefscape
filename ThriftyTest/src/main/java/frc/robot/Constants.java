// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
        public static final int motorID = 59;
        public static final int cancoderPort = 51;

        public static final int l_elevatorID = 51; // Left Elevator CanID
        public static final int r_elevatorID = 52; // Right Elevator CanID

        public static final int forwardLimitChannelID = 0;
        public static final int reverseLimitChannelID = 1;

        public static final double forwardSoftLimit = 2;

        public static final double supplyCurrentLimit = 40;

        public static final double rotorToSensorRatio = 1;
        public static final double sensorToMechanismRatio = 1;

        public static final InvertedValue invertedValue = InvertedValue.Clockwise_Positive;

        public static final double k_momentInertia = 0.2188; // SI units
        public static final double k_gearRatio = 125.0;

        public static final double maxHeight = 4;

        public static final double tolerance = maxHeight * 0.01; // 1% tolerance

        private static final Vector<N2> k_stateSpaceStdDevs = VecBuilder.fill(0.1, 0.3);

        private static final Vector<N2> qelms = VecBuilder.fill(0.0001, 0.1);
        private static final Vector<N1> relms = VecBuilder.fill(4.0);

        private static final LinearSystem<N2, N1, N2> k_plant = LinearSystemId
                .createDCMotorSystem(TalonFXConstants.TalonFXDCMotor, k_momentInertia, k_gearRatio);

        public static final StateSpaceConfig<N2, N1, N2> k_config = new StateSpaceConfig<N2, N1, N2>(
                k_plant,
                k_stateSpaceStdDevs,
                VecBuilder.fill(TalonFXConstants.positionStdDevs, TalonFXConstants.velocityStdDevs),
                qelms,
                relms,
                Nat.N2(),
                Nat.N2(),
                tolerance,
                "Elevator");

        public static final double k_absoluteSensorRange = 0.5;
        public static final SensorDirectionValue k_cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        public static final double k_encoderOffset = 0.324707;

        public static final double stow = 0;
        public static final double processor = 0.25;
        public static final double L1 = 0.5;
        public static final double L2 = 1.5;
        public static final double L3 = 2.5;
        public static final double L4 = 3.5;
        public static final double net = 4;
    }

    public static final class PivotConstants {
        public static final int pivotMotorID = 59;
        public static final int EncoderID = 51;
        public static final double encoderOffset = 0.324707;

        public static final double rotorToSensorRatio = 125;
        public static final double sensorToMechanismRatio = 1.0;

        public static final int algaePivotID = 62; // Algae CANID 


        public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        public static final double forwardSoftLimitThreshold = 0.085693;
        public static final double reverseSoftLimitThreshold = 0;

        public static final int forwardSoftLimitThresholdChannelID = 3; // Placed Arbitrarily
        public static final int reverseSoftLimitThresholdChannelID = 4; // Placed Arbitrarily

        public static final double radiansAtZero = Math.toRadians(30);
        public static final double radiansAtMax = Math.toRadians(58);

        public static final double pivotManualUpSpeed = 0.3;
        public static final double pivotManualDownSpeed = -0.1;

        public static final double absoluteSensorRange = 0.5;

        public static final double tolerance = forwardSoftLimitThreshold * 0.01; // 1% tolerance

        public static final double groundPickup = 0.5;
        public static final double processor = 0.25;
        public static final double reefPickup = 0.5;
        public static final double net = 0.0;
        public static final double stow = 0;


        private static final Vector<N2> k_stateSpaceStdDevs = VecBuilder.fill(0.1, 0.3);

        private static final Vector<N2> qelms = VecBuilder.fill(0.0001, 0.1);
        private static final Vector<N1> relms = VecBuilder.fill(4.0);

        public static final double k_momentInertia = 0.2188; // SI units
        public static final double k_gearRatio = 125.0;

        private static final LinearSystem<N2, N1, N2> k_plant = LinearSystemId
                .createDCMotorSystem(TalonFXConstants.TalonFXDCMotor, k_momentInertia, k_gearRatio);

        public static final StateSpaceConfig<N2, N1, N2> k_config = new StateSpaceConfig<N2, N1, N2>(
                k_plant,
                k_stateSpaceStdDevs,
                VecBuilder.fill(TalonFXConstants.positionStdDevs, TalonFXConstants.velocityStdDevs),
                qelms,
                relms,
                Nat.N2(),
                Nat.N2(),
                tolerance,
                "Pivot");

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