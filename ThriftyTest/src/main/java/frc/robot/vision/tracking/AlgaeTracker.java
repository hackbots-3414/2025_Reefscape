package frc.robot.vision.tracking;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.utils.OnboardLogger;
import frc.robot.vision.CameraIO;
import frc.robot.vision.CameraIO.CameraIOInputs;
import frc.robot.vision.CameraIOHardware;
import frc.robot.vision.CameraIOInputsLogger;

public class AlgaeTracker implements Runnable {

  private final OnboardLogger m_ologger = new OnboardLogger("Tracking");

  public static final boolean enabled = TrackingConstants.kEnabled;

  public record ObjectTrackingStatus(Rotation2d yaw, double time, Optional<Pose2d> pose) {
    public boolean isExpired() {
      return Timer.getTimestamp() - time() >= TrackingConstants.kExpirationTime.in(Seconds);
    }
  }

  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(AlgaeTracker.class);

  private final CameraIO m_io;
  private final CameraIOInputs m_inputs = new CameraIOInputs();
  private final CameraIOInputsLogger m_inputsLogger;

  private final Consumer<ObjectTrackingStatus> m_action;

  private final Supplier<Pose2d> m_robotPose;

  private Pose2d m_lastSeenAlgae = Pose2d.kZero;

  public AlgaeTracker(Supplier<Pose2d> robotPose, Consumer<ObjectTrackingStatus> action) {
    if (Robot.isSimulation()) {
      m_io = new CameraIOTrackingSim(
          TrackingConstants.kCameraName,
          TrackingConstants.kRobotToCamera,
          robotPose);
    } else {
      m_io = new CameraIOHardware(TrackingConstants.kCameraName);
    }
    m_action = action;
    m_robotPose = robotPose;
    m_inputsLogger = new CameraIOInputsLogger(m_inputs, TrackingConstants.kCameraName);
    m_ologger.registerPose("Last Seen", () -> m_lastSeenAlgae);
  }

  public void run() {
    m_io.updateInputs(m_inputs);
    m_inputsLogger.log();

    List<PhotonPipelineResult> results = m_inputs.unreadResults;

    results.forEach(result -> {
      if (!result.hasTargets()) {
        return;
      }
      PhotonTrackedTarget target = result.getBestTarget();
      Rotation2d yaw = Rotation2d.fromDegrees(-result.getBestTarget().yaw);

      Optional<Double> estimatedDistance = estimateDistance(target);

      estimatedDistance.ifPresentOrElse(dist -> {
        // Determine camera-relative position of algae
        Transform2d cameraOffset =
            new Transform2d(yaw.getCos(), yaw.getSin(), Rotation2d.kZero).times(dist);
        // Determine robot-relative position of algae
        Pose2d robot = m_robotPose.get();
        Pose2d camera = robot
            .transformBy(new Transform2d(
                TrackingConstants.kRobotToCamera.getTranslation().toTranslation2d(),
                TrackingConstants.kRobotToCamera.getRotation().toRotation2d()));
        Pose2d algae = camera.transformBy(cameraOffset);
        m_lastSeenAlgae = algae;
        m_action.accept(new ObjectTrackingStatus(
            algae.relativeTo(robot).getTranslation().getAngle(),
            Timer.getTimestamp(),
            Optional.of(algae)));
      }, () -> {
        m_action.accept(new ObjectTrackingStatus(
            yaw,
            Timer.getTimestamp(),
            Optional.empty()));
      });

    });
    m_ologger.log();
  }

  private Optional<Double> estimateDistance(PhotonTrackedTarget target) {
    if (!TrackingConstants.kDistanceEstimationEnabled) {
      return Optional.empty();
    }
    if (target.pitch < 0) {
      return estimateDistance(target.pitch, TrackingConstants.kGroundAlgaeHeight);
    }
    if (target.pitch > 0) {
      return estimateDistance(target.pitch, TrackingConstants.kLollipopAlgaeHeight);
    }
    // The algae could not be detected
    return Optional.empty();
  }

  private Optional<Double> estimateDistance(double pitch, Distance targetHeight) {
    double diff = TrackingConstants.kRobotToCamera.getZ() - targetHeight.in(Meters);
    return Optional.of(Math.abs(diff / Math.sin(Units.degreesToRadians(pitch))));
  }
}
