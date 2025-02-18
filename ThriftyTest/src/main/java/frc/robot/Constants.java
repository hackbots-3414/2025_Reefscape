package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.PS5Controller.Axis;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import frc.robot.generated.TunerConstants;
import frc.robot.stateSpace.StateSpaceConfig;
import frc.robot.utils.Shape;

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

    public static class IDConstants {
        public static final int elevatorLeft = 51;
        public static final int elevatorRight = 52;
        public static final int elevatorEncoder = 53;

        public static final int pivotMotor = 57;
        public static final int pivotEncoder = 58;

        public static final int coralLeft = 55;
        public static final int coralRight = 56;
        public static final int candi = 59;

        public static final int climbLeft = 1;
        public static final int climbRight = 2;

        public static final int algaeMotor = 60;
    }

    public static class SimConstants {
        public static final double k_simPeriodic = 0.005;
    }

    public static class RobotConstants {
        public static final Time globalCanTimeout = Milliseconds.of(20); // 20 milliseconds

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
        public static final PIDConstants k_translationPID = new PIDConstants(15.0, 0.0, 0.0);
        public static final PIDConstants k_rotationPID = new PIDConstants(5.0, 0.0, 0.0);

        public static final double k_maxLinearSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double k_maxAngularSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        
        public static final double k_maxLinearAcceleration = k_maxLinearSpeed * 2;
        public static final double k_maxAngularAcceleration = k_maxAngularSpeed * 2;

        public static final double k_elevatorHeightLinearVelocityGain = -0.357; // for every 1 rotation elevator up, subtract X: 1 mps at max elevator
        public static final double k_elevatorHeightLinearAccelerationGain = k_elevatorHeightLinearVelocityGain * 2;
        public static final double k_elevatorHeightAngularVelocityGain = -0.0446; // for every 1 rotation elevator up, subtract X: 0.25 rps at max elevator
        public static final double k_elevatorHeightAngularAccelerationGain = k_elevatorHeightAngularVelocityGain * 2;

        public static final double k_closedLoopOverrideToleranceTranslation = 0.02;
        public static final double k_closedLoopOverrideToleranceRotation = 0.02;
    }


    public static class ButtonBindingConstants {
        public static enum DriverChoice {DRAGONREINS, BACKUP;}
        public static enum ButtonBoardChoice {BUTTONBOARD, BACKUP;}

        public static final DriverChoice driverChoice = DriverChoice.DRAGONREINS;
        public static final ButtonBoardChoice buttonBoardChoice = ButtonBoardChoice.BACKUP;

        public static final int driverPort = 0;
        public static final int buttonBoardPort = 1;


        public static class DragonReins {
            public static final int xAxis = 1;
            public static final int yAxis = 0;
            public static final int rotAxis = 2;

            public static final boolean flipX = true;
            public static final boolean flipY = false;
            public static final boolean flipRot = true;

            public static final int enableOpenLoop = 3;
            public static final int resetHeading = 1;
        }

        public static class BackupDriver {
            public static final int xAxis = Axis.kLeftY.value;
            public static final int yAxis = Axis.kLeftX.value;
            public static final int rotAxis = Axis.kRightY.value;

            public static final boolean flipX = true;
            public static final boolean flipY = false;
            public static final boolean flipRot = true;

            public static final int enableOpenLoop = Button.kSquare.value;
            public static final int resetHeading = Button.kCircle.value;
        }

        public static class ButtonBoard {
            public static final int safetySwitch = 17;

            // WHEN SAFETY ON - AUTOMATION BASED
            public static final int L1Auto = 1;
            public static final int L2Auto = 2;
            public static final int L3Auto = 3;
            public static final int L4Auto = 4;
            public static final int R1Auto = 5;
            public static final int R2Auto = 6;
            public static final int R3Auto = 7;
            public static final int R4Auto = 8;

            public static final int lowAlgaeAuto = 9;
            public static final int highAlgaeAuto = 10;
            public static final int groundAlgaeAuto = 11;
            public static final int processorAuto = 12;
            public static final int netAuto = 13;

            public static final int leftIntake = 14;
            public static final int rightIntake = 15;

            public static final int climbAuto = 16;

            // WHEN SAFETY OFF - MANUAL STUFF
            public static final int l1Score = 1;
            public static final int l2Score = 2;
            public static final int l3Score = 3;
            public static final int l4Score = 4;

            public static final int manualElevatorUp = 5;
            public static final int manualElevatorDown = 6;
            public static final int manualPivotUp = 7;
            public static final int manualPivotDown = 8;

            public static final int lowAlgae = 9;
            public static final int highAlgae = 10;
            public static final int groundAlgae = 11;
            public static final int processor = 12;
            public static final int net = 13;

            public static final int intake = 14;
            public static final int spitPiece = 15;

            public static final int climb = 16;
        }

        public static class ButtonBoardAlternate {
            public static final int safetySwitch = Button.kTouchpad.value;

            // WHEN SAFETY ON - AUTOMATION BASED
            public static final int L1 = 0; // POV
            public static final int L2 = 270; // POV
            public static final int L3 = 90; // POV
            public static final int L4 = 180; // POV
            public static final int leftReef = Button.kSquare.value;
            public static final int rightReef = Button.kCircle.value;

            public static final int lowAlgaeAuto = Button.kCross.value;
            public static final int highAlgaeAuto = Button.kTriangle.value;
            public static final int groundAlgaeAuto = 180; // POV
            public static final int processorAuto = 90; // POV
            public static final int netAuto = 0; // POV
            public static final int algaeModeButton = Button.kOptions.value;
            
            public static final int leftIntake = Button.kL1.value; // LB
            public static final int rightIntake = Button.kR1.value; // RB
            
            public static final int climbAuto = Button.kCreate.value;

            // WHEN SAFETY OFF - MANUAL STUFF
            public static final int l1Score = 0;
            public static final int l2Score = 270;
            public static final int l3Score = 90;
            public static final int l4Score = 180;

            public static final int manualElevatorUp = Axis.kLeftY.value;
            public static final int manualElevatorDown = Axis.kLeftY.value;
            public static final int manualPivotUp = Axis.kRightY.value;
            public static final int manualPivotDown = Axis.kRightY.value;

            public static final int lowAlgae = Button.kCross.value;
            public static final int highAlgae = Button.kTriangle.value;
            public static final int groundAlgae = Button.kSquare.value;
            public static final int processor = Button.kCircle.value;
            public static final int net = Button.kR1.value;

            public static final int intake = Button.kL1.value; // LB
            public static final int spitPiece = Button.kL2.value; // LT

            public static final int climb = Button.kCreate.value;
        }
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
        // Stop using vision after X time
        public static final double k_visionTimeout = 0.5;
    }

    public static class FieldConstants {
        public static final Distance k_fieldWidth = Meters.of(8.05);
        public static final Distance k_fieldLength = Meters.of(17.55);
        public static final Translation2d reefCenter = new Translation2d(4.5, 4.0);
    }

    public static final class StateSpaceConstants {
        public static final double k_dt = 0.01; // fast state space, please!
        public static final double k_maxVoltage = 12.0;
    }

    public static final class AutonConstants {
        public static final boolean useSuperAuton = false;

        public static final int numWaypoints = 5;
        public static double pathplannerMinRange = 0.5;
        public static double overrideTolerance = 0.05;
        public static double degreeTolerance = 2;
    }

    public static final class CanRangeConstants {

        public static final CANrangeConfiguration k_canRangeConfig = new CANrangeConfiguration();
        // .withFovParams(null)
        // .withProximityParams(null)
        // .withToFParams(null);

        public static final int k_filterWindow = 5; // 5 measurements
    }

    public static final class ElevatorConstants {
        public static final boolean invertLeftMotorFollower = true;

        public static final double forwardSoftLimit = 11.2;
        public static final double reverseSoftLimit = 0;

        public static final double supplyCurrentLimit = 40;

        public static final double rotorToSensorRatio = 5.2;
        public static final double sensorToMechanismRatio = 1;

        public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;
        
        public static final double gearRatio = rotorToSensorRatio * sensorToMechanismRatio;

        public static final double stage1Mass = Units.lbsToKilograms(5.402);
        public static final double stage2Mass = Units.lbsToKilograms(4.819);
        public static final double carriageMass = Units.lbsToKilograms(3.084);
        public static final double coralMechanismMass = Units.lbsToKilograms(8.173); // includes coral
        public static final double algaeMechanismMass = Units.lbsToKilograms(8.359);

        public static final double netMass = stage1Mass + stage2Mass + carriageMass + coralMechanismMass + algaeMechanismMass; // Mass of the elevator carriage
        public static final double drumRadius = Units.inchesToMeters(2.256 / 2); // Radius of the elevator drum

        public static final double momentOfInertia = netMass * Math.pow(drumRadius, 2);

        public static final double tolerance = forwardSoftLimit * 0.05; // 5% tolerance

        private static final Vector<N2> stateSpaceStandardDeviations = VecBuilder.fill(0.1, 0.03);

        private static final Vector<N2> qelms = VecBuilder.fill(0.02, 0.1);
        private static final Vector<N1> relms = VecBuilder.fill(4.0);
        
        // public static final LinearSystem<N2, N1, N2> stateSpacePlant  = LinearSystemId
        //         .createDCMotorSystem(
        //             TalonFXConstants.TalonFXDCMotor,
        //             momentOfInertia,
        //             gearRatio
        //         );

        public static final LinearSystem<N2, N1, N2> stateSpacePlant = LinearSystemId
            .createElevatorSystem(TalonFXConstants.TalonFXDCMotor, netMass, drumRadius, gearRatio);

        public static final StateSpaceConfig<N2, N1, N2> stateSpaceConfig = new StateSpaceConfig<>(
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
        public static final double encoderOffset = -0.427979;

        public static final double stow = 0.0;
        public static final double processor = 0.125;
        public static final double L1 = 2;
        public static final double L2 = 4;
        public static final double L3 = 6;
        public static final double L4 = 8;
        public static final double net = 1.95;
        public static final double reefLower = 0.5; // arbitrary, meters
        public static final double reefUpper = 1.5; // arbitrary, meters

        public static final double manualUpSpeed = 0.2;
        public static final double manualDownSpeed = -0.2;

        public static final double maxSpeed = 10; // cancoder rotations per second
        public static final double accelerationMultiplier = 3;

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
                        .withFeedbackRemoteSensorID(IDConstants.elevatorEncoder)
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANcoder)
                        .withRotorToSensorRatio(rotorToSensorRatio)
                        .withSensorToMechanismRatio(sensorToMechanismRatio))

                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(supplyCurrentLimit))

                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(forwardSoftLimit)
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(reverseSoftLimit)
                        .withForwardSoftLimitEnable(true))

                .withSlot0(new Slot0Configs()
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(5)
                        .withKI(0)
                        .withKD(0)
                        .withKS(0)
                        .withKV(3.59 * (drumRadius * 2 * Math.PI))
                        .withKA(0.05 * (drumRadius * 2 * Math.PI))
                        .withKG(0.42))

                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(maxSpeed)
                        .withMotionMagicAcceleration(maxSpeed * accelerationMultiplier)
                        .withMotionMagicJerk(maxSpeed * accelerationMultiplier * 10));
    }

    public static final class PivotConstants {
        public static final double encoderOffset = -0.558594;

        public static final double rotorToSensorRatio = 64.0 / 14.0; 
        public static final double sensorToMechanismRatio = 32.0 / 14.0; 

        public static final InvertedValue invertMotor = InvertedValue.Clockwise_Positive;
        public static final SensorDirectionValue invertEncoder = SensorDirectionValue.CounterClockwise_Positive;

        public static final double forwardSoftLimitThreshold = 0.65;
        public static final double reverseSoftLimitThreshold = 0;

        public static final double radiansAtMax = forwardSoftLimitThreshold;
        public static final double radiansAtZero = 0;

        public static final double absoluteSensorRange = 1;

        public static final double supplyCurrentLimit = 20;

        public static final double tolerance = forwardSoftLimitThreshold * 0.01; // 1% tolerance

        public static final double groundPickup = 0.62;
        public static final double processor = 0.2;
        public static final double reefPickup = 0.5;
        public static final double reefExtract = 0.4;
        public static final double net = 0.6;
        public static final double stow = 0.03;

        public static final double manualUpSpeed = 0.1;
        public static final double manualDownSpeed = -0.1;

        private static final Vector<N2> stateSpaceStandardDeviation = VecBuilder.fill(0.1, 0.3);

        private static final Vector<N2> qelms = VecBuilder.fill(0.002, 0.1);
        private static final Vector<N1> relms = VecBuilder.fill(0.5);

        public static final double momentOfIntertia = 0.14622;
        public static final double gearRatio = rotorToSensorRatio * sensorToMechanismRatio;

        public static final LinearSystem<N2, N1, N2> stateSpacePlant = LinearSystemId
                .createSingleJointedArmSystem(TalonFXConstants.TalonFXDCMotor, momentOfIntertia, gearRatio);

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
                        .withFeedbackRemoteSensorID(IDConstants.pivotEncoder)
                        .withFeedbackRotorOffset(encoderOffset)
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                        .withRotorToSensorRatio(rotorToSensorRatio)
                        .withSensorToMechanismRatio(sensorToMechanismRatio))

                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(supplyCurrentLimit))

                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(forwardSoftLimitThreshold)
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(reverseSoftLimitThreshold)
                        .withReverseSoftLimitEnable(true));

        public static final double armLength = 0.443;
    }

    public static class CoralConstants {
        public static final double intakeVoltage = 12;
        public static final double ejectVoltage = 12;
        public static final boolean rightMotorInvert = true;

        public static final double supplyCurrentLimit = 20;

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))

                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(supplyCurrentLimit));

        public static final CANdiConfiguration candiConfig = new CANdiConfiguration()
                .withDigitalInputs(new DigitalInputsConfigs()
                        .withS1CloseState(S1CloseStateValue.CloseWhenHigh)
                        .withS2CloseState(S2CloseStateValue.CloseWhenHigh));

        public static double intakeTimeout = 1;
    }

    public static final class ClimberConstants {
        public static final boolean rightMotorInvert = true;
        public static final double climberUpVolts = 1.0; //FIXME figure out actual values for the climber voltage.
        public static final double climberCurrentLimit = 80.0;
        public static final InvertedValue invertMotor = InvertedValue.CounterClockwise_Positive;

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(invertMotor))

                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(climberCurrentLimit));
    }

    public static final class AlgaeRollerConstants {
        public static final double intakeVoltage = 12; //FIXME tune for actual robot
        public static final double ejectVoltage = -1; //FIXME tune for actual robot

        public static final double torqueCurrentThreshold = 30; //FIXME tune for actual robot

        public static final double supplyCurrentLimit = 20.0;
        public static final double statorCurrentLimit = 40.0;

        public static final double holdVoltage = 1;
        public static final double k_updateObjectPeriodSeconds = 0.200; // 200 milliseconds
        public static final InvertedValue invertMotor = InvertedValue.Clockwise_Positive;
        public static final double algaeEjectTime = 0.3;
        public static final double reefPickupSafetyDistance = 1.75;

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(invertMotor))

                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(supplyCurrentLimit));
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
        CLOSEHP(new Pose2d(1.217, 7.012, Rotation2d.fromDegrees(-55))),
        PROCESSOR(new Pose2d(6.0, 0.5, Rotation2d.fromDegrees(-90))),
        NET(new Pose2d(7.7, 6.0, Rotation2d.fromDegrees(0)));

        public Pose2d value;

        private ScoringLocations(Pose2d value) {
            this.value = value;
        }
    }

    public enum ScoringLocationsLeft {
        A(ScoringLocations.A.value),
        C(ScoringLocations.C.value),
        E(ScoringLocations.E.value),
        G(ScoringLocations.G.value),
        I(ScoringLocations.I.value),
        K(ScoringLocations.K.value);

        public Pose2d value;

        private ScoringLocationsLeft(Pose2d value) {
            this.value = value;
        }
    }

    public enum ScoringLocationsRight {
        B(ScoringLocations.B.value),
        D(ScoringLocations.D.value),
        F(ScoringLocations.F.value),
        H(ScoringLocations.H.value),
        J(ScoringLocations.J.value),
        L(ScoringLocations.L.value);

        public Pose2d value;

        private ScoringLocationsRight(Pose2d value) {
            this.value = value;
        }
    }

    public enum ScoringLocationsMiddle {
        AB(ScoringLocations.A.value.interpolate(ScoringLocations.B.value, 0.5)),
        CD(ScoringLocations.C.value.interpolate(ScoringLocations.D.value, 0.5)),
        EF(ScoringLocations.E.value.interpolate(ScoringLocations.F.value, 0.5)),
        GH(ScoringLocations.G.value.interpolate(ScoringLocations.H.value, 0.5)),
        IJ(ScoringLocations.I.value.interpolate(ScoringLocations.J.value, 0.5)),
        KL(ScoringLocations.K.value.interpolate(ScoringLocations.L.value, 0.5));

        public Pose2d value;

        private ScoringLocationsMiddle(Pose2d value) {
            this.value = value;
        }
    }

    public enum ClimbLocations {
        WALL(new Pose2d(8.5, 7.26, Rotation2d.fromDegrees(0))),
        MIDDLE(new Pose2d(8.5, 6.1, Rotation2d.fromDegrees(0))),
        CENTER(new Pose2d(8.5, 5.0, Rotation2d.fromDegrees(0)));

        public Pose2d value;

        private ClimbLocations(Pose2d value) {
            this.value = value;
        }
    }

    public enum ReefClipLocations {
        LEFT, RIGHT;
    }

    public static final class CommandBounds {
        // 1 robot of space around the entire reef
        public static final List<Translation2d> reef = List.of(
            new Translation2d(5.85, 3.2),
            new Translation2d(5.85, 4.8),
            new Translation2d(4.46, 5.66),
            new Translation2d(3.06, 4.8),
            new Translation2d(3.06, 3.2),
            new Translation2d(4.46, 2.40)
        );
        public static final Shape reefBounds = Shape.fromUnsortedVertices(reef);

        // 1.5 robot of space away from the opposite alliance barge side intake
        public static final List<Translation2d> leftIntake = List.of(
            new Translation2d(0.0, 1.25),
            new Translation2d(1.7, 0.0),
            new Translation2d(3.2, 0.0),
            new Translation2d(0.0, 2.35)
        );
        public static final Shape leftIntakeBounds = Shape.fromUnsortedVertices(leftIntake);

        // 1.5 robot of space away from the same alliance barge side intake
        public static final Shape rightIntakeBounds = Shape.flipHotdog(leftIntakeBounds);

        // processor where we score
        public static final List<Translation2d> oppositeAllianceProcessor = List.of(
            new Translation2d(5.5, 0.0),
            new Translation2d(6.5, 0.0),
            new Translation2d(6.5, 1),
            new Translation2d(5.5, 1)
        );
        public static final Shape processorBounds = Shape.fromUnsortedVertices(oppositeAllianceProcessor);

        // net where we score
        public static final List<Translation2d> net = List.of(
            new Translation2d(7.2, 4.25),
            new Translation2d(10.3, 4.25),
            new Translation2d(10.3, 8),
            new Translation2d(7.2, 8)
        );
        public static final Shape netBounds = Shape.fromUnsortedVertices(net);

        public static Map<String, Shape> displayBounds = Map.ofEntries(
            Map.entry("Blue Alliance Reef", reefBounds),
            Map.entry("Blue Alliance Net", netBounds),
            Map.entry("Blue Alliance Left Intake", leftIntakeBounds),
            Map.entry("Blue Alliance Right Intake", rightIntakeBounds),
            Map.entry("Blue Alliance Processor", processorBounds)
        );
    }
}
