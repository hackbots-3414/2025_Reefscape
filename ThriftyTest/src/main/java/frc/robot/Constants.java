package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.stateSpace.StateSpaceConfig;

/*
PLEASE READ:

To ensure consistency throughout the code, the same coordinate system is used
here as is specified in WPILib's documentation.

To read it all, check this out:
https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html

TL;DR:
We use NWU axes convention.
This means that, when viewed from above, the North, West, and then upwards will
correspond to +X, +Y, and +Z respectively.

Example:
           +X
            ^
            |
        |-front-|
        |       |
+Y <--- |       |
        |       |
        |-------|

And +Z is upwards, so it wouldn't show here.
*/

public class Constants {
    public static class RobotConstants {
        public static final double k_robotX = Units.inchesToMeters(30.0);
        public static final double k_robotY = Units.inchesToMeters(30.0);

        public static final double k_cameraOffsetX = 0.75 * k_robotX / 2.0;    
        public static final double k_cameraOffsetY = 0.75 * k_robotY / 2.0;
        public static final double k_cameraHeight = Units.inchesToMeters(6.0);
        public static final double k_cameraBackHeight = Units.inchesToMeters(12.0);
        public static final double k_cameraPitch = -Units.degreesToRadians(27.5);
        public static final double k_backCameraPitch = -Units.degreesToRadians(51.0);
        public static final double k_cameraYaw = Units.degreesToRadians(35.0);
        public static final double k_cameraBackYaw = Units.degreesToRadians(45.0);
    }
    public static class DriveConstants {
        public static final PIDConstants k_translationPID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants k_rotationPID = new PIDConstants(5.0, 0.0, 0.0);
    }
    public static class VisionConstants {
        public static final String k_estimationName = "estimation";
        // aliases
        private static final double x = RobotConstants.k_cameraOffsetX;
        private static final double y = RobotConstants.k_cameraOffsetY;
        private static final double z = RobotConstants.k_cameraHeight;
        private static final double zBack = RobotConstants.k_cameraBackHeight;
        private static final double pitch = RobotConstants.k_cameraPitch;
        private static final double backPitch = RobotConstants.k_backCameraPitch;

        private static final double yaw = RobotConstants.k_cameraYaw;
        private static final double backYaw = RobotConstants.k_cameraBackYaw;

        // The camera names
        public static Map<String, Transform3d> cameras = Map.ofEntries(
            Map.entry("0", new Transform3d(
                new Translation3d(x, y, z),
                new Rotation3d(0, pitch, -yaw)
            )),
            Map.entry("1", new Transform3d(
                new Translation3d(x, y, z),
                new Rotation3d(0, pitch, Math.PI - yaw)
            )),
            Map.entry("2", new Transform3d(
                new Translation3d(-x, y, zBack),
                new Rotation3d(0, pitch, yaw)
            )),
            Map.entry("3", new Transform3d(
                new Translation3d(-x, y, zBack),
                new Rotation3d(0, backPitch, backYaw - Math.PI)
            )),
            Map.entry("4", new Transform3d(
                new Translation3d(-x, -y, zBack),
                new Rotation3d(0, backPitch, Math.PI - backYaw)
            )),
            Map.entry("5", new Transform3d(
                new Translation3d(-x, -y, zBack),
                new Rotation3d(0, pitch, -yaw)
            )),
            Map.entry("6", new Transform3d(
                new Translation3d(x, -y, z),
                new Rotation3d(0, pitch, yaw - Math.PI)
            )),
            Map.entry("7", new Transform3d(
                new Translation3d(x, -y, z),
                new Rotation3d(0, pitch, yaw)
            ))
        );
        // The tick time for each pose estimator to run
        public static final double k_periodic = 0.02;
        // The maximum number of results (per camera) we expect to see per tick
        public static final int k_maxResults = 2;
        // The maximum tolerated latency, in seconds.
        public static final double k_latencyThreshold = 0.75;
        // The maximum tolerated ambiguity value.
        public static final double k_AmbiguityThreshold = 0.2;
        // The farthest out off a field a pose estimate can say we are
        // (in each dimension separately)
        public static final Distance k_XYMargin = Meters.of(0.5);
        // The maximum distance from 0 that a camera's pose can report
        public static final Distance k_ZMargin = Meters.of(1.5);
        // Some configuration variables:
        public static final boolean k_useStdDevs = true;
        public static final double k_distanceMultiplier = 7.0;
        public static final double k_noisyDistance = 4.0;
        public static final double k_ambiguityMultiplier = 0.4;
        public static final double k_ambiguityShifter = 0.2;
        public static final double k_targetMultiplier = 10;
        // Stats about the camera for simulation
        public static final int k_resWidth = 320;
        public static final int k_resHeight = 240;
        public static final Rotation2d k_fov = Rotation2d.fromDegrees(82.0);
        // Simulated error:
        public static final Time k_avgLatency = Milliseconds.of(18);
        public static final Time k_latencyStdDev = Milliseconds.of(5);
        public static final double k_avgErr = 0.03;
        public static final double k_errStdDev = 0.02;
    }
    public static class FieldConstants {
        public static final Distance k_fieldWidth = Meters.of(8.05);
        public static final Distance k_fieldLength = Meters.of(17.55);
    }

    public static final class StateSpaceConstants {
        public static final double k_dt = 0.02;
        public static final double k_maxVoltage = 4.0;
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

    public enum ScoringLocations {
        A(new Pose2d(3.16, 4.19, Rotation2d.fromDegrees(0))),
        B(new Pose2d(3.16, 3.875, Rotation2d.fromDegrees(0))),
        C(new Pose2d(3.675, 3, Rotation2d.fromDegrees(60))),
        D(new Pose2d(4, 2.78, Rotation2d.fromDegrees(60))),
        E(new Pose2d(5, 2.8, Rotation2d.fromDegrees(120))),
        F(new Pose2d(5.3, 3, Rotation2d.fromDegrees(120))),
        G(new Pose2d(5.8, 3.85, Rotation2d.fromDegrees(180))),
        H(new Pose2d(5.8, 4.2, Rotation2d.fromDegrees(180))),
        I(new Pose2d(5.3, 5.1, Rotation2d.fromDegrees(-120))),
        J(new Pose2d(5, 5.25, Rotation2d.fromDegrees(-120))),
        K(new Pose2d(4, 5.25, Rotation2d.fromDegrees(-60))),
        L(new Pose2d(3.675, 5.1, Rotation2d.fromDegrees(-60))),
        FARHP(new Pose2d(1.194, 1.026, Rotation2d.fromDegrees(55))),
        CLOSEHP(new Pose2d(1.217, 7.012, Rotation2d.fromDegrees(-55)));
        
        public Pose2d value;

        private ScoringLocations(Pose2d value) {
            this.value = value;
        }
    }

}