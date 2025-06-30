package frc.robot.vision.tracking;

import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotObserver;

public class AlgaeTracker implements Runnable {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(AlgaeTracker.class);

  private final PhotonCamera camera = new PhotonCamera("camera");
  private final VisionSystemSim visionSim = new VisionSystemSim("test");

  private final TargetModel target = new TargetModel(0.5);
  private final Pose3d targetPose = new Pose3d(4, 4, 0.5, Rotation3d.kZero);
  private final VisionTargetSim targetSim = new VisionTargetSim(targetPose, target);

  private final SimCameraProperties cameraProp = new SimCameraProperties();

  private final PhotonCameraSim simCamera;

  private final Supplier<Pose2d> robotPose;
  private final Consumer<ObjectTrackingStatus> action;

  public AlgaeTracker(Supplier<Pose2d> robotPose, Consumer<ObjectTrackingStatus> action) {
    this.robotPose = robotPose;
    this.action = action;

    visionSim.addVisionTargets(targetSim);

    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    cameraProp.setFPS(24);
    cameraProp.setAvgLatencyMs(17);
    cameraProp.setLatencyStdDevMs(8);
    cameraProp.setCalibError(0.75, 0.05);

    simCamera = new PhotonCameraSim(camera, cameraProp);
    simCamera.enableDrawWireframe(true);

    visionSim.addCamera(simCamera, Transform3d.kZero);

    RobotObserver.getField().getObject("Algae").setPose(targetPose.toPose2d());
  }

  public void run() {
    // update target location
    targetSim.setPose(new Pose3d(RobotObserver.getField().getObject("Algae").getPose()));

    visionSim.update(robotPose.get());

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    results.forEach(result -> {
      if (!result.hasTargets()) {
        return;
      }
      double yaw = result.getBestTarget().getYaw();

      action.accept(new ObjectTrackingStatus(Rotation2d.fromDegrees(yaw), Timer.getTimestamp()));
    });
  }

  public record ObjectTrackingStatus(Rotation2d yaw, double time) {
    public boolean isExpired(Time expirationTime) {
      return Timer.getTimestamp() - time() >= expirationTime.in(Seconds);
    }
  }
}
