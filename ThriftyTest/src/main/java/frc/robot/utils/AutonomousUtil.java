package frc.robot.utils;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotObserver;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonomousUtil {
    @SuppressWarnings("unused")
    private static final Logger m_logger = LoggerFactory.getLogger(RobotContainer.class);

    private static CommandSwerveDrivetrain m_drivetrain;

    public static void initializePathPlanner(CommandSwerveDrivetrain drivetrain) {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    drivetrain::getPose, // Robot pose supplier
                    drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> drivetrain.driveWithChassisSpeeds(speeds),
                    DriveConstants.k_pathplannerHolonomicDriveController,
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

            m_drivetrain = drivetrain;

            PathPlannerLogging.setLogActivePathCallback(poses -> RobotObserver.getField().getObject("Pathfind Trajectory").setPoses(poses));
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    private static final PathConstraints pathFindConstraints = new PathConstraints(DriveConstants.k_maxLinearSpeed, DriveConstants.k_maxLinearAcceleration, DriveConstants.k_maxAngularSpeed, DriveConstants.k_maxAngularAcceleration);
    private static final PathConstraints finalAlignConstraints = new PathConstraints(DriveConstants.k_maxAlignLinearSpeed, DriveConstants.k_maxAlignLinearAcceleration, DriveConstants.k_maxAlignAngularSpeed, DriveConstants.k_maxAlignAngularAcceleration);

    private static Command pathFindThenPreciseAlign(Pose2d pose) {
        Pose2d startPose = new Pose2d(
            (Math.cos(pose.getRotation().getRadians()) * -1) + pose.getX(),
            (Math.sin(pose.getRotation().getRadians()) * -1) + pose.getY(),
            pose.getRotation()
        );
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, pose);
        PathPlannerPath path = new PathPlannerPath(waypoints, finalAlignConstraints, new IdealStartingState(DriveConstants.k_maxAlignLinearSpeed.in(MetersPerSecond), pose.getRotation()), new GoalEndState(0, pose.getRotation()));
        return AutoBuilder.pathfindThenFollowPath(path, pathFindConstraints);
    }
    
    public static Command pathFinder(Pose2d pose) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotObserver.setReefMode(true)),
            pathFindThenPreciseAlign(pose),
            // new DriveToPointCommand(FieldUtils.flipPose(pose), m_drivetrain)
            new InstantCommand(() -> RobotObserver.setReefMode(false))
        );
    }

    public static Command driveToPoint(Pose2d pose, CommandSwerveDrivetrain drivetrain) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> RobotObserver.setReefMode(true)),
            new DriveToPointCommand(FieldUtils.flipPose(pose), drivetrain),
            new InstantCommand(() -> RobotObserver.setReefMode(false))
        );
    }

    public static Command generateRoutineWithCommands(CommandSwerveDrivetrain drivetrain, Pose2d desiredPickupLocation, Pose2d[] poses, Command[] scoringCommands, Supplier<Command> intakeCommand) {
        SequentialCommandGroup routine = new SequentialCommandGroup();
        for (int i = 0; i < scoringCommands.length; i++) {
            if (i != 0) {
                routine.addCommands(pathFinder(desiredPickupLocation));
                routine.addCommands(intakeCommand.get());
            }
            routine.addCommands(pathFinder(poses[i]));
            routine.addCommands(scoringCommands[i]);
            routine.addCommands(pathFinder(desiredPickupLocation));
            routine.addCommands(intakeCommand.get());
        }

        return routine;
    }

    private static ArrayList<Command> onTheFlyCommands = new ArrayList<>();

    public static void queuePathWithCommand(Pose2d pose, Supplier<Command> command) {
        onTheFlyCommands.add(pathFinder(pose));
        onTheFlyCommands.add(command.get());
    }

    public static void queueClosest(Supplier<Command> scoreSupplier, List<Pose2d> scoringLocationList) {
        queuePathWithCommand(clip(scoringLocationList), scoreSupplier);
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

    public static Command followPath(String pathName) {
        System.out.println("CONSTRUCTED PATH WITH NAME:" + pathName);
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

}
