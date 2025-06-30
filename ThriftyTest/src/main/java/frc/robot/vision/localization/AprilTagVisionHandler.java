package frc.robot.vision.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Robot;
import frc.robot.utils.LoopTimer;
import frc.robot.vision.CameraIO;
import frc.robot.vision.CameraIOHardware;

public class AprilTagVisionHandler implements AutoCloseable {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(AprilTagVisionHandler.class);

  private final LoopTimer m_loopTimer;

  private final Supplier<Pose2d> m_poseSupplier;
  private final Consumer<TimestampedPoseEstimate> m_consumer;

  private final Notifier m_notifier;
  private final List<SingleInputPoseEstimator> m_estimators = new ArrayList<>();
  private final MultiInputFilter m_filter;

  private final AprilTagVisionLogger m_esimateLogger;

  public AprilTagVisionHandler(Supplier<Pose2d> poseSupplier, Consumer<TimestampedPoseEstimate> callback) {
    m_poseSupplier = poseSupplier;
    m_consumer = callback;
    m_filter = new MultiInputFilter();
    m_esimateLogger = new AprilTagVisionLogger();
    setupCameras();
    m_notifier = new Notifier(this::updateEstimators);
    m_loopTimer = new LoopTimer("Vision");
  }

  private void setupCameras() {
    for (Map.Entry<String, Transform3d> entry : AprilTagVisionConstants.kCameras.entrySet()) {
      String cameraName = entry.getKey();
      Transform3d robotToCamera = entry.getValue();
      CameraIO io;
      if (Robot.isSimulation()) {
        io = new CameraIOAprilTagSim(cameraName, robotToCamera, m_poseSupplier);
      } else {
        io = new CameraIOHardware(cameraName, robotToCamera);
      }
      SingleInputPoseEstimator estimator = new SingleInputPoseEstimator(
          m_filter,
          io,
          this::addEstimate);
      m_estimators.add(estimator);
    }
  }

  private void updateEstimators() {
    m_loopTimer.reset();
    m_filter.clear();
    for (SingleInputPoseEstimator estimator : m_estimators) {
      estimator.refresh(m_poseSupplier.get());
    }

    for (SingleInputPoseEstimator estimator : m_estimators) {
      estimator.run();
    }
    // finish logging
    m_esimateLogger.log();
    m_loopTimer.log();
  }

  public void startThread() {
    m_notifier.startPeriodic(AprilTagVisionConstants.kPeriodic);
  }

  private void addEstimate(TimestampedPoseEstimate estimate) {
    if (DriverStation.isDisabled()) {
      if (!m_filter.verify(estimate.pose())) {
        return;
      }
    }
    m_consumer.accept(estimate);
    m_esimateLogger.addEstimate(estimate);
  }

  @Override
  public void close() {
    m_notifier.close();
  }
}
