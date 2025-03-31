package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.Set;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
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
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import frc.robot.generated.TunerConstants;
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
        public static final int leftRange = 8;
        public static final int rightRange = 7;

        public static final int elevatorLeft = 51;
        public static final int elevatorRight = 52;
        public static final int elevatorCANrange = 53;

        public static final int pivot = 57;

        public static final int coralLeft = 55;
        public static final int coralRight = 56;
        public static final int coralCANrange = 59;
        public static final int upperCANrange = 58;

        public static final int frontIR = 2;
        public static final int rearIR = 3;

        public static final int climbLeft = 1;
        public static final int climbRight = 2;

        public static final int algae = 60;

        public static final int candle1 = 5; 
        public static final int candle2 = 6;

        public static final int servo = 9;

        public static final int climbEncoder = 9;
    }

    public static class SimConstants {
        public static final double k_simPeriodic = 0.005;
    }

    public static class RobotConstants {
        public static final Time globalCanTimeout = Milliseconds.of(20); // 20 milliseconds
    }
    
    public static class DriveConstants {
        public static final PIDConstants k_translationPID = new PIDConstants(2, 0.0, 0.0); // 0.18836
        public static final PIDConstants k_rotationPID = new PIDConstants(1.5, 0.0, 0.0); // 0.17119
        public static final PIDConstants k_driveToPointRotationPID = new PIDConstants(4, 0.0, 0.0); // 0.17119

        public static final double kMaxAccelerationPerpendicularToTarget = 3.0;
        public static final double kMaxAccelerationTowardsTarget = 3.0;

        public static final PPHolonomicDriveController k_pathplannerHolonomicDriveController = new PPHolonomicDriveController(k_translationPID, k_rotationPID);

        public static final double k_maxTeleopLinearSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double k_maxTeleopAngularSpeed = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

        public static final double k_driveToPointSpeed = 4.0;
        public static final double k_driveToPointAcceleration = 1.0;

        public static final LinearVelocity k_maxLinearSpeed = MetersPerSecond.of(4);
        public static final LinearAcceleration k_maxLinearAcceleration = MetersPerSecondPerSecond.of(3);
        public static final AngularVelocity k_maxAngularSpeed = RotationsPerSecond.of(2);
        public static final AngularAcceleration k_maxAngularAcceleration = RotationsPerSecondPerSecond.of(2);

        public static final LinearVelocity k_maxAlignLinearSpeed = MetersPerSecond.of(1.0);
        public static final LinearAcceleration k_maxAlignLinearAcceleration = MetersPerSecondPerSecond.of(1);
        public static final AngularVelocity k_maxAlignAngularSpeed = RotationsPerSecond.of(1);
        public static final AngularAcceleration k_maxAlignAngularAcceleration = RotationsPerSecondPerSecond.of(1);

        public static final double k_maxRotationalSpeed = k_maxLinearSpeed.in(MetersPerSecond) / (TunerConstants.kWheelRadius.in(Meters) * 2 * Math.PI); // lin speed / circumference = rot speed

        public static final double k_elevatorHeightLinearVelocityGain = -0.357; // for every 1 rotation elevator up, subtract X: 1 mps at max elevator
        public static final double k_elevatorHeightLinearAccelerationGain = k_elevatorHeightLinearVelocityGain * 2;
        public static final double k_elevatorHeightAngularVelocityGain = -0.0446; // for every 1 rotation elevator up, subtract X: 0.25 rps at max elevator
        public static final double k_elevatorHeightAngularAccelerationGain = k_elevatorHeightAngularVelocityGain * 2;

        public static final double k_closedLoopOverrideToleranceTranslation = 0.05;
        public static final double k_closedLoopOverrideToleranceRotation = 0.05;

        public static final double rangeZero = 0.175;
        public static final double rangeMax = 0.3;

        // These are the constraints solely used by the DriveToPoint commands
        public static final Constraints k_driveToPointConstraints = new Constraints(
            k_driveToPointSpeed,
            k_driveToPointAcceleration
        );
        // This one is as well, however it is only used in auton
        public static final Constraints k_rotationConstraints = new Constraints(
            k_maxAngularSpeed.in(RadiansPerSecond),
            k_maxAngularAcceleration.in(RadiansPerSecondPerSecond)
        );
    }

    public static class ButtonBindingConstants {
        public static enum DriverChoice {DRAGONREINS, BACKUP;}
        public static enum ButtonBoardChoice {PS5, KEYBOARD;}

        public static final DriverChoice driverChoice = DriverChoice.DRAGONREINS;
        public static final ButtonBoardChoice buttonBoardChoice = ButtonBoardChoice.PS5;

        public static final String dragonReinsName = "spark";
        public static final String driverBackupName = "inter";

        public static final String ps5Name = "dual";

        public static final int driverPort = 0;
        public static final int buttonBoardPort = 1;

        public static class DragonReins {
            public static final int xAxis = 1;
            public static final int yAxis = 0;
            public static final int rotAxis = 3;

            public static final boolean flipX = false;
            public static final boolean flipY = true;
            public static final boolean flipRot = false;

            public static final int resetHeading = 1;

            public static final double deadband = 0.01;
        }

        public static class PS5 {
            public static final int L1 = 180; // POV
            public static final int L2 = 270; // POV
            public static final int L3 = 90; // POV
            public static final int L4 = 0; // POV

            public static final int ejectCoral = Button.kL2.value;
            
            public static final int leftReef = Button.kSquare.value;
            public static final int rightReef = Button.kCircle.value;

            public static final int lowAlgae = Button.kCross.value;
            public static final int highAlgae = Button.kTriangle.value;
            public static final int groundAlgae = 180; // POV
            public static final int processor = 90; // POV
            public static final int highGround = 270; // POV
            public static final int net = 0; // POV
            public static final int algaeModeButton = Button.kR2.value; // R2
            
            public static final int leftIntake = Button.kL1.value; // LB
            public static final int rightIntake = Button.kR1.value; // RB
            
            public static final int climbReady = Button.kCreate.value;
            public static final int climb = Button.kOptions.value;

            public static final int stow = Button.kPS.value;

            public static final int intake = Button.kL1.value; // LB

            public static final int zeroElevator = 15; // old safety mode button (little bar below PS button)
        }
    
        public static class ButtonBoardKeyboard {
            // WHEN SAFETY ON - AUTOMATION BASED
            public static final int L1 = 1;
            public static final int L2 = 2;
            public static final int L3 = 3;
            public static final int L4 = 4;

            public static final int A = 5;
            public static final int B = 6;
            public static final int C = 7;
            public static final int D = 8;
            public static final int E = 9;
            public static final int F = 10;
            public static final int G = 11;
            public static final int H = 12;
            public static final int I = 13;
            public static final int J = 14;
            public static final int K = 15;
            public static final int L = 16;

            public static final int lowAlgae = 17;
            public static final int highAlgae = 18;
            public static final int groundAlgae = 19;
            public static final int processor = 20;
            public static final int net = 21;

            public static final int leftIntake = 22;
            public static final int rightIntake = 23;

            public static final int climb = 24;

            public static final int cancelAuto = 25;
        }
    }

    public static class VisionConstants {
        public static final boolean enableVision = true;
        public static final boolean k_enableLogging = false;

        public static final double k_rotationCoefficient = Math.PI * 20;
        public static final double k_translationCoefficient = 0.1;

        public static AprilTagFieldLayout k_layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final String k_estimationName = "estimation";

        public static final String k_logPath = "/home/lvuser/logs/vision";
        public static final String k_simLogPath = "logs/vision";

        private static final double k_moduleHeight = 0.190;

        private static final double k_tightPitch = -Units.degreesToRadians(22.5);
        private static final double k_widePitch = -Units.degreesToRadians(25.0);

        private static final double k_tightYaw = Units.degreesToRadians(37.0); // this doesn't seem right
        private static final double k_wideYaw = Units.degreesToRadians(-7.0);

        // The camera names
        public static Map<String, Transform3d> fakecameras = Map.ofEntries(
            Map.entry("test", new Transform3d())
        );
        public static Map<String, Transform3d> cameras = Map.ofEntries(
            Map.entry("cam1", new Transform3d( // left tight
                new Translation3d(0.256, 0.289, k_moduleHeight),
                new Rotation3d(0, k_tightPitch, -k_tightYaw)
            )),
            Map.entry("cam2", new Transform3d( // left wide
                new Translation3d(0.337, 0.331, k_moduleHeight),
                new Rotation3d(0, k_widePitch, -k_wideYaw)
            )),
            Map.entry("cam3", new Transform3d( // right wide
                new Translation3d(0.337, -0.331, k_moduleHeight),
                new Rotation3d(0, k_widePitch, k_wideYaw)
            )),
            Map.entry("cam4", new Transform3d( // right tight
                new Translation3d(0.256, -0.289, k_moduleHeight),
                new Rotation3d(0, k_tightPitch, k_tightYaw)
            ))
        );

        public static final String k_leftAlignName = "cam1";
        public static final String k_rightAlignName = "cam4";

        // The tick time for each pose estimator to run
        public static final double k_periodic = 0.02;
        // The maximum number of results (per camera)
        public static final double k_expectedResults = 10;
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
        public static final double k_distanceMultiplier = 5.0;
        public static final double k_noisyDistance = 4.0;
        public static final double k_ambiguityMultiplier = 0.4;
        public static final double k_ambiguityShifter = 0.2;
        public static final double k_targetMultiplier = 80;
        public static final double k_differenceThreshold = 0.10;
        public static final double k_differenceMultiplier = 200.0;
        public static final double k_latencyMultiplier = 1.3;

        public static final double k_headingThreshold = Units.degreesToRadians(3);

        // Stats about the camera for simulation
        public static final int k_resWidth = 320;
        public static final int k_resHeight = 240;
        public static final Rotation2d k_fov = Rotation2d.fromDegrees(82.0);

        // Simulated error:
        public static final Time k_avgLatency = Milliseconds.of(18);
        public static final Time k_latencyStdDev = Milliseconds.of(2);
        public static final double k_avgErr = 0.18;
        public static final double k_errStdDev = 0.02;

        // Stop using vision after X time
        public static final double k_visionTimeout = 0.5;

        // reef tag ids (single tag only)
        public static final Set<Integer> k_reefIds = Set.of(
            6,  7,  8,  9,  10, 11, // red tags
            17, 18, 19, 20, 21, 22 // blue tags
        );
    }

    public static class FieldConstants {
        public static final Distance k_fieldWidth = Meters.of(8.05);
        public static final Distance k_fieldLength = Meters.of(17.55);
        public static final Translation2d reefCenter = new Translation2d(4.5, 4.0);
        public static final double k_reefReady = 2.1;
    }

    public static final class StateSpaceConstants {
        public static final double k_dt = 0.01; // fast state space, please!
        public static final double k_maxVoltage = 12.0;
    }

    public static final class AutonConstants {
        public static double translationTolerance = 0.03; // 0.04
        public static Angle rotationTolerance = Degrees.of(2);

        public static double driveToPointMaxDistance = 1.5; // beyond X meters, command will insta end
        public static double stage2Distance = 1;
    }

    public static final class CanRangeConstants {

        public static final CANrangeConfiguration k_canRangeConfig = new CANrangeConfiguration()
        .withFovParams(new FovParamsConfigs()
            .withFOVRangeX(7)
            .withFOVRangeY(7)
        )
        .withToFParams(new ToFParamsConfigs()
            .withUpdateMode(UpdateModeValue.ShortRange100Hz)
        );
        // .withProximityParams(null)
        public static final double farAlignedDistanceMeters = 0.18; 
        public static final double tolerance = 0.2; // 20% tolerance
        public static final double closeAlignedDistanceMeters = 0.12; 
        public static final int k_filterWindow = 5; // 5 measurements
    }

    public static final class ElevatorConstants {
        public static final boolean enable = true;

        public static final boolean invertLeftMotorFollower = true;

        public static final double supplyCurrentLimit = 100;
        public static final double k_zeroCurrentThreshold = 23.5;

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
        // approx. 0.02865

        public static final double momentOfInertia = netMass * Math.pow(drumRadius, 2);

        public static final LinearSystem<N2, N1, N2> stateSpacePlant = LinearSystemId
            .createElevatorSystem(KrakenX60FOCConstants.KrakenX60FOCMotor, netMass, drumRadius, gearRatio);

        public static final double absoluteSensorRange = 0.5;
        public static final SensorDirectionValue invertEncoder = SensorDirectionValue.CounterClockwise_Positive;
        public static final double encoderOffset = 0.291015625 ; //0.490234375

        public static final double metersToRotations = 1 / (drumRadius * 2 * Math.PI);
        // approx 7.96

        public static final boolean enableCANRange = true;

        public static final double rangeDistanceGain = 64; // how much higher, per unit of range

        /* Please note:
         * The maximum height of the elevator (in inches) was calculated to be 80.44 inches.
         * Accounting for e rror, we really never should set a setpoint higher than 79 inches (how we chose the net height)
         */

        public static final double inch = Units.inchesToMeters(1) * metersToRotations;

        public static final double groundIntake = 0;
        public static final double highGroundIntake = Units.inchesToMeters(12.0) * metersToRotations;
        public static final double stow = 0.425;
        public static final double processor = 0;
        // public static final double L1 = stow + 3.5 * inch;
        public static final double L2 = 4.016 + 2 * inch; // 35.5
        public static final double L1 = L2 - 2.5 * inch;
        public static final double L3 = 7.257 - 4 * inch; // 50.5
        public static final double L4 = 9.757 + 0.3 * inch;
        public static final double net = 9.31 + 4 * inch; // 67 - short, // 72 - long
        public static final double reefLower = 2;
        public static final double reefUpper = 4.5;
        public static final double prep = L2;

        public static final double forwardSoftLimit = 11.15;
        public static final double reverseSoftLimit = 0;

        public static final double unsafeRange = L2 + 2 * inch;

        public static final double tolerance = 0.06;

        public static final double k_maxCanCompensation = 2 * inch;

        public static final double manualUpSpeed = 0.2;
        public static final double manualDownSpeed = -0.3;

        public static final double maxSpeedUp = 32; // 16
        public static final double maxAccelerationUp = 48; // 48
        public static final double maxJerkUp = 480; // 480

        public static final double maxSpeedDown = 10; // 10
        public static final double maxAccelerationDown = 30; // 30
        public static final double maxJerkDown = 300; // 300

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

        public static final CANrangeConfiguration kCANrangeConfig = new CANrangeConfiguration()
            .withFovParams(new FovParamsConfigs()
                .withFOVRangeX(6.75)
                .withFOVRangeY(6.75))
            .withProximityParams(new ProximityParamsConfigs()
                .withMinSignalStrengthForValidMeasurement(3500)
                .withProximityThreshold(0.12));

        public static final Time kRangeDebounceTime = Seconds.of(0.06);
    }

    public static final class PivotConstants {
        public static final double encoderOffset = 0.665283203125;

        public static final double rotorOffset = 0.344;

        public static final double rotorToSensorRatio = 64.0 / 14.0; 
        public static final double sensorToMechanismRatio = 32.0 / 14.0; 

        public static final InvertedValue invertMotor = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue invertEncoder = SensorDirectionValue.Clockwise_Positive;

        public static final double forwardSoftLimitThreshold = 0.359;
        public static final double reverseSoftLimitThreshold = 0.0;

        public static final double radiansAtMax = forwardSoftLimitThreshold;
        public static final double radiansAtZero = 0;

        public static final double absoluteSensorRange = 0.5;

        public static final double supplyCurrentLimit = 40;

        public static final double tolerance = 0.03;

        public static final double groundPickup = 0.0669;
        public static final double processor = 0.085;
        public static final double reefPickup = 0.2;
        public static final double reefExtract = 0.281;
        public static final double net = 0.342;
        public static final double stow = 0.342;

        public static final double manualUpSpeed = 0.1;
        public static final double manualDownSpeed = -0.1;

        public static final double momentOfIntertia = 0.14622;
        public static final double gearRatio = rotorToSensorRatio * sensorToMechanismRatio;

        public static final LinearSystem<N2, N1, N2> stateSpacePlant = LinearSystemId
                .createSingleJointedArmSystem(TalonFXConstants.TalonFXDCMotor, momentOfIntertia, gearRatio);

        public static final double maxSpeed = 1.5; // cancoder rotations per second
        public static final double accelerationMultiplier = 2;

        public static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withAbsoluteSensorDiscontinuityPoint(absoluteSensorRange)
                        .withSensorDirection(invertEncoder)
                        .withMagnetOffset(encoderOffset));

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(invertMotor))

                // .withFeedback(new FeedbackConfigs()
                //         .withFeedbackRemoteSensorID(IDConstants.pivotEncoder)
                //         .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                //         .withRotorToSensorRatio(rotorToSensorRatio)
                //         .withSensorToMechanismRatio(sensorToMechanismRatio))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(gearRatio))

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
                        .withKP(15)
                        .withKI(0)
                        .withKD(0)
                        .withKS(0)
                        .withKV(1.3)
                        .withKA(0.12)
                        .withKG(0.625))
                .withSlot1(new Slot1Configs()
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKP(20)
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

        public static final double armLength = 0.443;
    }

    public static class CoralConstants {
        public static final double intakeVoltage = 2.4;
        public static final double retractVoltage = -3.5;
        public static final double ejectVoltage = 5;

        public static final double l1EjectVoltage = 2.5;
        public static final double l2EjectVoltage = 4.0; // 5.1
        public static final double l3EjectVoltage = 4.0; // 5.1
        public static final double l4EjectVoltage = 5.5;

        public static final double rangeDistanceGain = 13; // how many more volts, per unit of range

        public static final double spitOutVoltage = -6;
        public static final double fastEjectVoltage = -10;

        public static final double l1LeftEjectVoltage = 8;
        public static final double l1RightEjectVoltage = 6;

        public static final boolean rightMotorInvert = true;

        public static final double supplyCurrentLimit = 20.0;

        public static final double IRThreshold = 0.51;

        public static final boolean enableCANRange = true;

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

        public static final CANrangeConfiguration rangeConfig = new CANrangeConfiguration()
                .withFovParams(new FovParamsConfigs()
                        .withFOVRangeX(6.5)
                        .withFOVRangeY(6.5))
                .withProximityParams(new ProximityParamsConfigs()
                        .withMinSignalStrengthForValidMeasurement(15015)
                        .withProximityThreshold(0.1))
                .withToFParams(new ToFParamsConfigs()
                        .withUpdateMode(UpdateModeValue.ShortRange100Hz));
        
        public static final CANrangeConfiguration upperRangeConfig = new CANrangeConfiguration()
                .withFovParams(new FovParamsConfigs()
                        .withFOVRangeX(6.5)
                        .withFOVRangeY(15))
                .withProximityParams(new ProximityParamsConfigs()
                        .withMinSignalStrengthForValidMeasurement(2500)
                        .withProximityThreshold(0.65))
                .withToFParams(new ToFParamsConfigs()
                        .withUpdateMode(UpdateModeValue.ShortRange100Hz));

        public static double intakeTimeout = 1;
    }

    public static final class ClimberConstants {
        public static final boolean rightMotorInvert = true;
        public static final double climberUpVolts = 12.0; // 12.0
        public static final double climbDownVolts = -12.0;
        public static final double climbRollVolts = -4;

        public static final double climberCurrentLimit = 80.0;
        public static final InvertedValue invertMotor = InvertedValue.CounterClockwise_Positive;

        public static final double forwardSoftLimit = 0.0;
        public static final double reverseSoftLimit = -0.23;
        public static final double climbPosition = -0.115;

        public static final double encoderOffset = -0.01318359;
        public static final SensorDirectionValue invertEncoder = SensorDirectionValue.CounterClockwise_Positive;

        public static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withAbsoluteSensorDiscontinuityPoint(0.5)
                        .withSensorDirection(invertEncoder)
                        .withMagnetOffset(encoderOffset));

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()                        
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(invertMotor))

                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(climberCurrentLimit))

                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitThreshold(forwardSoftLimit)
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(reverseSoftLimit)
                    .withReverseSoftLimitEnable(true))

                .withFeedback(new FeedbackConfigs()
                        .withFeedbackRemoteSensorID(IDConstants.climbEncoder)
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));

        public static final double climbReadyRangeValue = 0.08;
        public static final double climbedRangeValue = 0.145;

        public static final double climbMaxEncoderValue = 63.833;
        public static final double climbReadyMaxEncoderValue = 90;
            
        public static final double k_openServoPosition = 0.0;
        public static final double k_closedServoPosition = 1.0;
        public static final double k_servoTolerance = 0.01;

        public static final double climbReadyTolerance = -0.001;
    }

    public static final class AlgaeRollerConstants {
        public static final double intakeVoltage = 12;
        public static final double ejectVoltage = -3.0; // 1.5
        public static final double processorEjectVoltage = -3.2;

        public static final double torqueCurrentThreshold = 75;

        public static final double supplyCurrentLimit = 25.0;

        public static final double holdVoltage = 2.5;
        public static final double k_updateObjectPeriodSeconds = 0.200; // 200 milliseconds
        public static final InvertedValue invertMotor = InvertedValue.Clockwise_Positive;
        public static final double algaeEjectTime = 0.6;
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
        public static final double nominalVoltageVolts = 12.0; // DC Volts
        public static final double stallTorqueNewtonMeters = 4.69; // Nm
        public static final double stallCurrentAmps = 257.0; // Amps
        public static final double freeCurrentAmps = 1.5; // Amps
        public static final double freeSpeedRadPerSec = 6380.0 * 2.0 * Math.PI / 60.0; // RPM * 2pi / 60 = Rad per
                                                                                       // second

        public static final double positionStdDevs = 1.0 / 2048.0; // rotations
        public static final double velocityStdDevs = 2.0 / 2048.0; // rotations

        public static final DCMotor TalonFXDCMotor = new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters,
                stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, 1);
    }

    public static final class KrakenX60Constants {
        public static final double nominalVoltageVolts = 12.0;
        public static final double stallTorqueNewtonMeters = 7.16;
        public static final double stallCurrentAmps = 374.38;
        public static final double freeCurrentAmps = 2.0;
        public static final double freeSpeedRadPerSec = Units.rotationsToRadians(6000);
        public static final double positionStdDevs = 1.0 / 2048.0;
        public static final double velocityStdDevs = 2.0 / 2048.0;

        public static final DCMotor KrakenX60Motor = new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters,
                stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, 1);
    }

    public static final class KrakenX60FOCConstants {
        public static final double nominalVoltageVolts = 12.0;
        public static final double stallTorqueNewtonMeters = 9.37;
        public static final double stallCurrentAmps = 483;
        public static final double freeCurrentAmps = 2.0;
        public static final double freeSpeedRadPerSec = Units.rotationsToRadians(5800);
        public static final double positionStdDevs = 1.0 / 2048.0;
        public static final double velocityStdDevs = 2.0 / 2048.0;

        public static final DCMotor KrakenX60FOCMotor = new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters,
                stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, 1);
    }

    public enum ScoringLocations {
        A(new Pose2d(3.197, 4.192, Rotation2d.fromDegrees(0))), // GOOD
        B(new Pose2d(3.178, 3.880, Rotation2d.fromDegrees(0))), // GOOD

        C(new Pose2d(3.703, 2.989, Rotation2d.fromDegrees(60))), // GOOD
        D(new Pose2d(3.971, 2.819, Rotation2d.fromDegrees(60))), // GOOD

        E(new Pose2d(4.991, 2.822, Rotation2d.fromDegrees(120))), // GOOD
        F(new Pose2d(5.275, 2.981, Rotation2d.fromDegrees(120))), // GOOD

        G(new Pose2d(5.783, 3.852, Rotation2d.fromDegrees(180))), // GOOD
        H(new Pose2d(5.794, 4.184, Rotation2d.fromDegrees(180))), // GOOD

        I(new Pose2d(5.282, 5.063, Rotation2d.fromDegrees(-120))), // GOOD
        J(new Pose2d(4.997, 5.240, Rotation2d.fromDegrees(-120))), // GOOD

        K(new Pose2d(3.988, 5.220, Rotation2d.fromDegrees(-60))), // GOOD
        L(new Pose2d(3.691, 5.072, Rotation2d.fromDegrees(-60))), // GOOD



        RIGHTHP(new Pose2d(1.227, 1.048, Rotation2d.fromDegrees(55))),
        LEFTHP(new Pose2d(1.227, 6.983, Rotation2d.fromDegrees(-55))),
        PROCESSOR(new Pose2d(6.0, 0.6, Rotation2d.fromDegrees(-90))),
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
            new Translation2d(2.729, 3.013),
            new Translation2d(4.498, 1.975),
            new Translation2d(6.242, 3.013),
            new Translation2d(6.242, 5.024),
            new Translation2d(4.498, 6.010),
            new Translation2d(2.729, 5.024)
        );
        public static final Shape reefBounds = Shape.fromUnsortedVertices(reef, "Reef");

        // 1.5 robot of space away from the opposite alliance barge side intake
        public static final List<Translation2d> leftIntake = List.of(
            new Translation2d(0.0, 1.25),
            new Translation2d(1.7, 0.0),
            new Translation2d(3.2, 0.0),
            new Translation2d(0.0, 2.35)
        );
        public static final Shape leftIntakeBounds = Shape.fromUnsortedVertices(leftIntake, "Left Intake");

        // 1.5 robot of space away from the same alliance barge side intake
        public static final Shape rightIntakeBounds = Shape.flipHotdog(leftIntakeBounds, "Right Intake");

        // processor where we score
        public static final List<Translation2d> oppositeAllianceProcessor = List.of(
            new Translation2d(5.5, 0.0),
            new Translation2d(6.5, 0.0),
            new Translation2d(6.5, 1),
            new Translation2d(5.5, 1)
        );
        public static final Shape processorBounds = Shape.fromUnsortedVertices(oppositeAllianceProcessor, "Processor");

        // net where we score
        public static final List<Translation2d> net = List.of(
            new Translation2d(7.2, 4.25),
            new Translation2d(10.3, 4.25),
            new Translation2d(10.3, 8),
            new Translation2d(7.2, 8)
        );
        public static final Shape netBounds = Shape.fromUnsortedVertices(net, "Net");

        public static final List<Translation2d> tooClose = List.of(
            new Translation2d(8.6 ,4.25 ),
            new Translation2d(11.7,4.25),
            new Translation2d(11.7,8),
            new Translation2d(8.6,8)

        );

        public static final Shape netTooCloseBounds = Shape.fromUnsortedVertices(tooClose, "NoNet");

        public static Map<String, Shape> displayBounds = Map.ofEntries(
            Map.entry("Blue Alliance Reef", reefBounds),
            Map.entry("Blue Alliance Net", netBounds),
            Map.entry("Blue Alliance Left Intake", leftIntakeBounds),
            Map.entry("Blue Alliance Right Intake", rightIntakeBounds),
            Map.entry("Blue Alliance Processor", processorBounds),
            Map.entry("Red Alliance Reef", reefBounds.flip()),
            Map.entry("Red Alliance Net", netBounds.flip()),
            Map.entry("Red Alliance Left Intake", leftIntakeBounds.flip()),
            Map.entry("Red Alliance Right Intake", rightIntakeBounds.flip()),
            Map.entry("Red Alliance Processor", processorBounds.flip())
        );
    }

    public static class LedConstants {
        public static final int numLED = 133;
        public static final double flashSpeed = 0.75;
        public static final double strobeSpeed = 0.1;
        public static final double endgameWarning = 30;
        public static final double endgameAlert = 15;
        public static final int funnelOffset = 8; // 8
        public static final int elevatorOffset = 95; // 94
        public static final int funnelNumLED = 87; // 85 
        public static final int elevatorNumLED = 40; // 40
        public static final int funnelOffset2 = 8; // 8
        public static final int elevatorOffset2 = 95; // 94
        public static final int funnelNumLED2 = 87; // 85
        public static final int elevatorNumLED2 = 40; // 40
    }

    public static class FFConstants {
        public static final double k_bargeX = 8.774176;
        public static final double k_radius = 1.3;
        public static final double k_decceleration = 1.2;
    }
}
