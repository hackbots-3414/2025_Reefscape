package frc.robot.vision.tracking;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotObserver;
import frc.robot.vision.CameraIO;

public class CameraIOTrackingSim implements CameraIO {
  private final String m_name;

  private final PhotonCamera m_camera;

  private final Supplier<Pose2d> m_robotPose;
  
  private final static Transform3d offset =
      new Transform3d(0, 0, TrackingConstants.kGroundAlgaeHeight.in(Meters), Rotation3d.kZero);

  // simulation stuff only happens once
  private static final VisionSystemSim visionSim = new VisionSystemSim("tracking");
  private static final TargetModel targetModel = new TargetModel(0.5);
  private static final Pose3d initialTargetPose = new Pose3d(2, 2, TrackingConstants.kLollipopAlgaeHeight.in(Meters), Rotation3d.kZero);
  private static final VisionTargetSim targetSim =
      new VisionTargetSim(initialTargetPose, targetModel);
  private static final SimCameraProperties simProps = new SimCameraProperties()
      .setFPS(TrackingConstants.kFPS)
      .setCalibration(
          TrackingConstants.kResWidth,
          TrackingConstants.kResHeight,
          TrackingConstants.kFOVDiag)
      .setCalibError(
          TrackingConstants.kCalibError,
          TrackingConstants.kCalibStdDevs)
      .setAvgLatencyMs(TrackingConstants.kLatency.in(Seconds))
      .setLatencyStdDevMs(TrackingConstants.kLatencyStdDevs.in(Seconds));

  // Add the target to the simulation
  static {
    visionSim.addVisionTargets(targetSim);
  }

  public CameraIOTrackingSim(
      String cameraName,
      Transform3d robotToCamera,
      Supplier<Pose2d> robotPose) {
    m_robotPose = robotPose;
    m_name = cameraName;
    SmartDashboard.putBoolean("Vision/" + m_name + " connected", true);

    m_camera = new PhotonCamera(cameraName);
    PhotonCameraSim cameraSim = new PhotonCameraSim(m_camera, simProps);
    visionSim.addCamera(cameraSim, robotToCamera);

    RobotObserver.getField().getObject("Tracked Object").setPose(initialTargetPose.toPose2d());
  }

  public void updateInputs(CameraIOInputs inputs) {
    inputs.connected = SmartDashboard.getBoolean("Vision/" + m_name + " connected", true);

    targetSim.setPose(
        new Pose3d(RobotObserver.getField().getObject("Tracked Object").getPose()).plus(offset));
    visionSim.update(m_robotPose.get());

    inputs.unreadResults = m_camera.getAllUnreadResults();
  }
}
