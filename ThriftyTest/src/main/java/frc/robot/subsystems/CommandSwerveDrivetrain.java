package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FFConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.AutonomousUtil;
import frc.robot.utils.FieldUtils;
import frc.robot.vision.TimestampedPoseEstimate;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private final Logger m_logger = LoggerFactory.getLogger(CommandSwerveDrivetrain.class);

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private Pose2d m_estimatedPose = new Pose2d();

    private double m_oldVisionTimestamp = -1;

    private boolean m_validPose = false;

    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;
    private final ApplyRobotSpeeds autoRequest = new ApplyRobotSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    private CANrange leftRange;
    private CANrange rightRange;

    private MedianFilter rangeFilter;
    private double rightRaw = -1;
    private double leftRaw = -1;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        setup();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        setup();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        setup();
    }

    private void setup() {
        AutonomousUtil.initializePathPlanner(this);
        if (Robot.isSimulation()) {
            startSimThread();
        }
        RobotObserver.setVisionValidSupplier(this::getVisionValid);
        RobotObserver.setPoseSupplier(this::getPose);
        RobotObserver.setVelocitySupplier(this::getVelocity);
        RobotObserver.setRangeDistanceSupplier(this::getRangeDistance);
        RobotObserver.setCompensationDistanceSupplier(this::getCompensationDistance);
        RobotObserver.setNoElevatorZoneSupplier(this::noElevatorZone);

        configureCANRange();
    }

    private void configureCANRange() {
        leftRange = new CANrange(IDConstants.leftRange);
        rightRange = new CANrange(IDConstants.rightRange);

        rangeFilter = new MedianFilter(5);
    }

    public void initializeSetpointGenerator(RobotConfig config) {
        setpointGenerator = new SwerveSetpointGenerator(config, Units.rotationsToRadians(DriveConstants.k_maxRotationalSpeed));

        ChassisSpeeds currSpeeds = getRobotRelativeSpeeds();
        SwerveModuleState[] currStates = getState().ModuleStates;
        previousSetpoint = new SwerveSetpoint(currSpeeds, currStates, DriveFeedforwards.zeros(config.numModules));
    }
    
    public Translation2d getVelocityComponents() {
        double vx = getRobotRelativeSpeeds().vxMetersPerSecond;
        double vy = getRobotRelativeSpeeds().vyMetersPerSecond;
        Rotation2d theta = getPose().getRotation();
        return new Translation2d(vx, vy).rotateBy(theta);
        
    }

    public double getVelocity() {
        Translation2d velo = getVelocityComponents();
        return velo.getNorm();
    }

    public Pose2d getPose() {
        return m_estimatedPose;
    }

    public Pose2d getNearestAntitarget() {
        return new Pose2d(FFConstants.k_bargeX, m_estimatedPose.getY(), new Rotation2d());
    }

    /**
     * returns the current pose, with red side poses flipped
     */
    public Pose2d getBluePose() {
        return FieldUtils.flipPose(m_estimatedPose);
    }

    public void zeroPose() {
        setPose(new Pose2d());
    }

    public void resetHeading() {
        setOperatorPerspectiveForward(getPose().getRotation());
    }

    public void setPose(Pose2d pose) {
        resetPose(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
        );

        setControl(autoRequest.withSpeeds(previousSetpoint.robotRelativeSpeeds()));
    }
    
    public void stop() {
        setControl(new SwerveRequest.SwerveDriveBrake());
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public double getRangeDistance() {
        double leftRaw = getRangeLeftDistance();
        double rightRaw = getRangeRightDistance();
        double leftValue = leftRaw > DriveConstants.rangeZero && leftRaw < DriveConstants.rangeMax ? leftRaw : DriveConstants.rangeZero;
        double rightValue = rightRaw > DriveConstants.rangeZero && rightRaw < DriveConstants.rangeMax ? rightRaw : DriveConstants.rangeZero;
        return rangeFilter.calculate(Math.min(leftValue, rightValue));
    }

    public double getRangeRightDistance() {
        return rightRaw;
    }

    public double getRangeLeftDistance() {
        return leftRaw;
    }

    public Optional<Double> getCompensationDistance() {
        double leftRaw = leftRange.getDistance().getValueAsDouble();
        boolean leftOk = leftRange.getIsDetected().getValue();
        double rightRaw = rightRange.getDistance().getValueAsDouble();
        boolean rightOk = rightRange.getIsDetected().getValue();
        if (!(rightOk || leftOk)) return Optional.empty();
        double measurement;
        if (!rightOk) {
            measurement = leftRaw;
        } else if (!leftOk) {
            measurement = rightRaw;
        } else {
            measurement = Math.min(leftRaw, rightRaw);
        }
        return Optional.of(rangeFilter.calculate(measurement));
    }

    @Override
    public void periodic() {
        m_estimatedPose = this.getState().Pose;

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        rightRaw = rightRange.getDistance().getValueAsDouble();
        leftRaw = leftRange.getDistance().getValueAsDouble();

        AutonomousUtil.handleQueue();

        handleVisionToggle();

        SmartDashboard.putString("REEF CLIP LOCATION", RobotObserver.getReefClipLocation().toString());
        SmartDashboard.putBoolean("REEF MODE ON", RobotObserver.getReefMode());
        SmartDashboard.putNumber("REEF ALING RANGE DISANCE", getRangeDistance());

         Translation2d v = getVelocityComponents();
         SmartDashboard.putNumber("vx", v.getX());
         SmartDashboard.putNumber("vy", v.getY());
        SmartDashboard.putNumber("LEFT", leftRange.getDistance().getValueAsDouble());
        SmartDashboard.putNumber("RIGHT", rightRange.getDistance().getValueAsDouble());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(SimConstants.k_simPeriodic);
    }

    private boolean getVisionValid() {
        return m_validPose;
    }

    private void handleVisionToggle() {
        if (m_oldVisionTimestamp >= 0) {
            m_validPose = Utils.getCurrentTimeSeconds() - m_oldVisionTimestamp < Constants.VisionConstants.k_visionTimeout;
        }
        SmartDashboard.putBoolean("VIABLE POSE", m_validPose);
    }

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
    }

    public void addPoseEstimate(TimestampedPoseEstimate estimate) {
        m_oldVisionTimestamp = estimate.timestamp();
        // This should NOT run in simulation!
        if (Robot.isSimulation()) {
            Transform2d error = getPose().minus(estimate.pose());
            m_logger.debug(
                "{} pose error: {}, {}\theading error: {}deg", 
                estimate.source(),
                error.getX(),
                error.getY(),
                error.getRotation().getDegrees()
            );
            return;
        }
        // Depending on our configs, we should use or not use the std devs
        if (Constants.VisionConstants.k_useStdDevs) {
            addVisionMeasurement(
                    estimate.pose(),
                    estimate.timestamp(),
                    estimate.stdDevs());
        } else {
            addVisionMeasurement(
                    estimate.pose(),
                    estimate.timestamp());
        }
    }

    public Command sysIdQuasistaticTranslation(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }

    public Command sysIdDynamicTranslation(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.dynamic(direction);
    }

    public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineSteer.quasistatic(direction);
    }

    public Command sysIdDynamicSteer(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineSteer.dynamic(direction);
    }

    public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.quasistatic(direction);
    }

    public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.dynamic(direction);
    }

    private boolean noElevatorZone() {
        return getNearestAntitarget().getTranslation().minus(m_estimatedPose.getTranslation()).getNorm() < FFConstants.k_radius;
    }
}
