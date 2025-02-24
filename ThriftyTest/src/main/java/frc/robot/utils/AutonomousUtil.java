package frc.robot.utils;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotObserver;
import frc.robot.commands.PathPlannerOverride;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonomousUtil {
    @SuppressWarnings("unused")
    private static final Logger m_logger = LoggerFactory.getLogger(RobotContainer.class);

    public static void initializePathPlanner(CommandSwerveDrivetrain drivetrain) {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    drivetrain::getPose, // Robot pose supplier
                    drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> drivetrain.driveWithChassisSpeeds(speeds),
                    new PPHolonomicDriveController(DriveConstants.k_translationPID, DriveConstants.k_rotationPID),
                    config, // The robot configuration
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    drivetrain); // Reference to this subsystem to set requirements

            drivetrain.initializeSetpointGenerator(config);

            PathPlannerLogging.setLogActivePathCallback(poses -> RobotObserver.getField().getObject("Pathfind Trajectory").setPoses(poses));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    private static PathConstraints constraints = new PathConstraints(DriveConstants.k_maxLinearSpeed, DriveConstants.k_maxLinearAcceleration, DriveConstants.k_maxAngularSpeed, DriveConstants.k_maxAngularAcceleration);

    public static Command pathFinder(Pose2d pose) {
        return AutoBuilder.pathfindToPoseFlipped(pose, constraints, 0);
    }

    public static Command generateRoutineWithCommands(CommandSwerveDrivetrain drivetrain, Pose2d desiredPickupLocation, Pose2d[] poses, Command[] scoringCommands, Supplier<Command> intakeCommand) {
        SequentialCommandGroup routine = new SequentialCommandGroup();
        for (int i = 0; i < poses.length; i++) {
            routine.addCommands(pathFinder(poses[i]));
            routine.addCommands(new PathPlannerOverride(FieldUtils.flipPose(poses[i]), drivetrain));
            routine.addCommands(scoringCommands[i]);
            routine.addCommands(pathFinder(desiredPickupLocation));
            routine.addCommands(new PathPlannerOverride(FieldUtils.flipPose(poses[i]), drivetrain));
            routine.addCommands(intakeCommand.get());
        }

        return routine;
    }

    private static ArrayList<Command> onTheFlyCommands = new ArrayList<>();

    public static void queuePathWithCommand(CommandSwerveDrivetrain drivetrain, Pose2d pose, Supplier<Command> command) {
        onTheFlyCommands.add(pathFinder(pose));
        onTheFlyCommands.add(new PathPlannerOverride(FieldUtils.flipPose(pose), drivetrain));
        onTheFlyCommands.add(command.get());
    }

    public static void queueClosest(CommandSwerveDrivetrain drivetrain, Supplier<Command> scoreSupplier, List<Pose2d> scoringLocationList) {
        queuePathWithCommand(drivetrain, clip(scoringLocationList), scoreSupplier);
    }

    public static Pose2d clip(List<Pose2d> list) {
        

        return FieldUtils.flipPose(RobotObserver.getPose()).nearest(list);
    }

    // ****** HANDLE THE QUEUE ********

    public static void clearQueue() {
        for (Command command : onTheFlyCommands) {
            command.cancel();
        }
        onTheFlyCommands = new ArrayList<>();
    }

    private static boolean ranCommand = false;

    public static void handleQueue() {
        if (!onTheFlyCommands.isEmpty()) {
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


    // ******** UNUSED METHODS; KEPT INCASE FUTURE IMPLEMENTATION NEEDED?? ********

    public static void queuePath(String pathName) {
        try {
            onTheFlyCommands.add(AutoBuilder.pathfindToPoseFlipped(
                    PathPlannerPath.fromPathFile(pathName).getStartingDifferentialPose(),
                    PathConstraints.unlimitedConstraints(12.0), 1));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void queuePath(Pose2d pose) {
        onTheFlyCommands.add(pathFinder(pose));
    }

    public static Command followPath(String pathName) {
        System.out.println("CONSTRUCTED PATH WITH NAME:" + pathName);
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static Command generateRoutine(Pose2d desiredPickupLocation, Pose2d[] poses) {
        SequentialCommandGroup routine = new SequentialCommandGroup();
        for (int i = 0; i < poses.length; i++) {
            if (i == 0) {
                routine.addCommands(pathFinder(poses[i]));
            } else {
                routine.addCommands(pathFinder(desiredPickupLocation));
                routine.addCommands(pathFinder(poses[i]));
            }
        }
        return routine;
    }

}
