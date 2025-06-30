package frc.robot.vision.tracking;

import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.vision.CameraIO;
import frc.robot.vision.CameraIOInputsLogger;
import frc.robot.vision.CameraIO.CameraIOInputs;
import frc.robot.vision.CameraIOHardware;

public class AlgaeTracker implements Runnable {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(AlgaeTracker.class);

  private final CameraIO m_io;
  private final CameraIOInputs m_inputs = new CameraIOInputs();
  private final CameraIOInputsLogger m_inputsLogger;

  private final Consumer<ObjectTrackingStatus> action;

  public AlgaeTracker(Supplier<Pose2d> robotPose, Consumer<ObjectTrackingStatus> action) {
    if (Robot.isSimulation()) {
      m_io = new CameraIOTrackingSim(
          TrackingConstants.kCameraName,
          TrackingConstants.kRobotToCamera,
          robotPose);
    } else {
      m_io = new CameraIOHardware(TrackingConstants.kCameraName);
    }
    this.action = action;
    m_inputsLogger = new CameraIOInputsLogger(m_inputs, TrackingConstants.kCameraName);

  }

  public void run() {
    m_io.updateInputs(m_inputs);
    m_inputsLogger.log();

    List<PhotonPipelineResult> results = m_inputs.unreadResults;

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
