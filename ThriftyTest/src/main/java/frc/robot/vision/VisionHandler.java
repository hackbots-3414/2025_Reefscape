package frc.robot.vision;

import static edu.wpi.first.units.Units.Milliseconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.RobotObserver;

public class VisionHandler implements AutoCloseable {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(VisionHandler.class);

  private final Consumer<TimestampedPoseEstimate> m_consumer;
  private final Supplier<Pose2d> m_poseSupplier;

  private final Notifier m_notifier;
  private final List<SingleInputPoseEstimator> m_estimators = new ArrayList<>();

  private final VisionSystemSim m_visionSim = new VisionSystemSim("main");
  private final SimCameraProperties m_simProps = new SimCameraProperties();

  private final Field2d m_field;

  private final VisionLogBuilder m_logBuilder;

  public VisionHandler(Supplier<Pose2d> poseSupplier, Consumer<TimestampedPoseEstimate> callback) {
    m_consumer = callback;
    m_poseSupplier = poseSupplier;
    m_visionSim.addAprilTags(VisionConstants.kTagLayout);
    setupProps();
    setupCameras();
    m_notifier = new Notifier(this::updateEstimators);
    if (Robot.isSimulation()) {
      m_field = m_visionSim.getDebugField();
    } else {
      m_field = new Field2d();
    }
    RobotObserver.setField(m_field);
    m_logBuilder = new VisionLogBuilder();
  }

  /**
   * Sets up the simulated camera properties with values that should reflect the real world
   * situation.
   */
  private void setupProps() {
    m_simProps.setCalibration(
        VisionConstants.kResWidth,
        VisionConstants.kResHeight,
        VisionConstants.kFOV);
    m_simProps.setAvgLatencyMs(VisionConstants.kAvgLatency.in(Milliseconds));
    m_simProps.setLatencyStdDevMs(VisionConstants.kLatencyStdDev.in(Milliseconds));
    m_simProps.setCalibError(VisionConstants.kAvgErr, VisionConstants.kErrStdDevs);
  }

  private void setupCameras() {
    for (Map.Entry<String, Transform3d> entry : VisionConstants.kCameras.entrySet()) {
      // it's easier to read this way:
      String cameraName = entry.getKey();
      Transform3d robotToCamera = entry.getValue();
      // initialze both real and simulated cameras
      PhotonCamera realCamera = new PhotonCamera(cameraName);
      if (Robot.isSimulation()) {
        PhotonCameraSim simCamera = new PhotonCameraSim(realCamera, m_simProps);
        m_visionSim.addCamera(simCamera, robotToCamera);

        // This is somewhat intensive (especially the first one) so we only
        // enable if the robot is in simulation mode.
        simCamera.enableDrawWireframe(true);
      }
      // we always need to add a vision estimator
      SingleInputPoseEstimator estimator = new SingleInputPoseEstimator(
          realCamera,
          robotToCamera,
          this::addEstimate);
      m_estimators.add(estimator);
    }
  }

  private void updateEstimators() {
    for (SingleInputPoseEstimator estimator : m_estimators) {
      estimator.refresh();
    }

    for (SingleInputPoseEstimator estimator : m_estimators) {
      estimator.run();
    }

    if (Robot.isSimulation()) {
      Pose2d currPose = m_poseSupplier.get();
      m_visionSim.update(currPose);
    }
    // finish logging
    m_logBuilder.log();
  }

  public void startThread() {
    m_notifier.startPeriodic(VisionConstants.kPeriodic);
  }

  private void addEstimate(TimestampedPoseEstimate estimate) {
    m_consumer.accept(estimate);
    // pose logging
    m_logBuilder.addEstimate(estimate);
  }

  @Override
  public void close() {
    m_notifier.close();
  }
}
