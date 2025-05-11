package frc.robot.utils;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotObserver;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonomousUtil {
  @SuppressWarnings("unused")
  private static final Logger m_logger = LoggerFactory.getLogger(AutonomousUtil.class);

  public static void initializePathPlanner(CommandSwerveDrivetrain drivetrain) {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          drivetrain::getPose, // Robot pose supplier
          drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a
                                 // starting pose)
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

      PathPlannerLogging.setLogActivePathCallback(
          poses -> RobotObserver.getField().getObject("Pathfind Trajectory").setPoses(poses));
    } catch (IOException | ParseException e) {
      e.printStackTrace();
      System.exit(1);
    }
  }
}
