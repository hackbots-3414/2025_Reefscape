package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
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
    private ApplyRobotSpeeds autoRequest = new ApplyRobotSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {startSimThread();}
        initializePathPlanner();
    }

    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,double odometryUpdateFrequency,SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {startSimThread();}
        initializePathPlanner();
    }

    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,double odometryUpdateFrequency,Matrix<N3, N1> odometryStandardDeviation,Matrix<N3, N1> visionStandardDeviation,SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {startSimThread();}
        initializePathPlanner();
    }

    public void initializePathPlanner() {
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setControl(autoRequest.withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(15.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(3.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this); // Reference to this subsystem to set requirements
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    public Pose2d getPose() {return estimatedPose;}
    public void setPose(Pose2d pose) {super.resetPose(pose);}
    public void zeroPose() {setPose(new Pose2d());}
    public ChassisSpeeds getRobotRelativeSpeeds() {return super.getState().Speeds;}
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {return run(() -> this.setControl(requestSupplier.get()));}


    private ArrayList<Command> onTheFlyCommands = new ArrayList<>();

    private Command getPathFindToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPoseFlipped(pose, PathConstraints.unlimitedConstraints(12.0), 1);
    }

    public Command generateAutonomousRoutineFromPoses(Pose2d desiredPickupLocation, Pose2d[] poses) {
        SequentialCommandGroup routine = new SequentialCommandGroup();
        for (int i = 0; i < poses.length; i++) {
            if (i==0) { 
                routine.addCommands(getPathFindToPose(poses[i]));
            } else {
                routine.addCommands(getPathFindToPose(desiredPickupLocation));
                routine.addCommands(getPathFindToPose(poses[i]));
            }
        }
        return routine;
    }

    public void sequenceOnTheFlyPaths(String pathName) {
        try {
            onTheFlyCommands.add(AutoBuilder.pathfindToPoseFlipped(PathPlannerPath.fromPathFile(pathName).getStartingDifferentialPose(), PathConstraints.unlimitedConstraints(12.0), 1));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @SuppressWarnings("CallToPrintStackTrace")
    public void sequenceOnTheFlyPaths(Pose2d pose) {
        try {
            onTheFlyCommands.add(getPathFindToPose(pose));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public Command getFollowPathCommandFromName(String pathName) {
        System.out.println("CONSTRUCTED PATH WQITH NAME:" + pathName);
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName)).andThen();
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public void resetOnTheFly() {
        for (Command command : onTheFlyCommands) {
            command.cancel();
        }
        onTheFlyCommands = new ArrayList<>();
    }

    private boolean ranCommand = false;

    public void handleOnTheFly() {
        if (onTheFlyCommands.size() > 0) {
            if (!ranCommand) {
                onTheFlyCommands.get(0).schedule();
                ranCommand = true;
            }
            if (!onTheFlyCommands.get(0).isScheduled() && ranCommand) {
                onTheFlyCommands.get(0).cancel();
                onTheFlyCommands.remove(0);
                ranCommand = false;
            }
        } else {
            ranCommand = false;
        }
    }

    @Override
    public void periodic() {
        estimatedPose = this.getState().Pose;

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
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
        handleOnTheFly();
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }



    // /* Swerve requests to apply during SysId characterization */
    // private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> setControl(m_translationCharacterization.withVolts(output)),
    //         null,
    //         this
    //     )
    // );

    // private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(7), // Use dynamic voltage of 7 V
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         volts -> setControl(m_steerCharacterization.withVolts(volts)),
    //         null,
    //         this
    //     )
    // );

    // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         /* This is in radians per second², but SysId only supports "volts per second" */
    //         Volts.of(Math.PI / 6).per(Second),
    //         /* This is in radians per second, but SysId only supports "volts" */
    //         Volts.of(Math.PI),
    //         null, // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> {
    //             /* output is actually radians per second, but SysId only supports "volts" */
    //             setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
    //             /* also log the requested output for SysId */
    //             SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
    //         },
    //         null,
    //         this
    //     )
    // );

    // private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {return m_sysIdRoutineToApply.quasistatic(direction);}
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {return m_sysIdRoutineToApply.dynamic(direction);}
}