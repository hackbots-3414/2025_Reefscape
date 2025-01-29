package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.AutonomousUtil;
import frc.robot.vision.TimestampedPoseEstimate;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private Logger m_logger = LoggerFactory.getLogger(CommandSwerveDrivetrain.class);
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private Pose2d estimatedPose = new Pose2d();

    private Field2d field = new Field2d();
    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        setup();
    }

    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,double odometryUpdateFrequency,SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        setup();
    }

    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,double odometryUpdateFrequency,Matrix<N3, N1> odometryStandardDeviation,Matrix<N3, N1> visionStandardDeviation,SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        setup();
    }

    public void setPose(Pose2d pose) {
        super.resetPose(pose);
    }

    public void zeroPose() {
        setPose(new Pose2d());
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        estimatedPose = this.getState().Pose;

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        field.setRobotPose(estimatedPose);
        
        AutonomousUtil.handleQueue();
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }



    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    private void setup() {
        AutonomousUtil.initializePathPlanner(this);
        if (Robot.isSimulation()) {
            startSimThread();
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
    }

    public void addPoseEstimate(TimestampedPoseEstimate estimate) {
        // This should NOT run in simulation!
        if (Robot.isSimulation()) return;
        // Depending on our configs, we should use or not use the std devs
        if (Constants.VisionConstants.k_useStdDevs) {
            addVisionMeasurement(
                estimate.pose(),
                estimate.timestamp(),
                estimate.stdDevs()
            );
        } else {
            addVisionMeasurement(
                estimate.pose(),
                estimate.timestamp()
            );
        }
    }

    public void resetView() {
        // set heading as this way!
        Rotation2d currentHeading = getPose().getRotation();
        setOperatorPerspectiveForward(currentHeading);
    }

    public Pose2d getPose() {
        return getState().Pose;
    }


    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
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
    
}
