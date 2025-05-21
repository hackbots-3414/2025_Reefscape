package frc.robot.vision;

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

public class VisionHandler implements AutoCloseable {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(VisionHandler.class);

  private final LoopTimer m_loopTimer;

  private final Supplier<Pose2d> m_poseSupplier;
  private final Consumer<TimestampedPoseEstimate> m_consumer;

  private final Notifier m_notifier;
  private final List<SingleInputPoseEstimator> m_estimators = new ArrayList<>();

  private final VisionLogBuilder m_logBuilder;

  private final VisionNetworkLogger m_networkLogger;

  private final MultiInputFilter m_filter;

  public VisionHandler(Supplier<Pose2d> poseSupplier, Consumer<TimestampedPoseEstimate> callback) {
    m_poseSupplier = poseSupplier;
    m_consumer = callback;
    m_filter = new MultiInputFilter();
    m_logBuilder = new VisionLogBuilder();
    m_networkLogger =
        new VisionNetworkLogger(Robot.isSimulation() || VisionConstants.kEnableNetworkLogging);
    setupCameras();
    m_notifier = new Notifier(this::updateEstimators);
    m_loopTimer = new LoopTimer("Vision");
  }

  private void setupCameras() {
    for (Map.Entry<String, Transform3d> entry : VisionConstants.kCameras.entrySet()) {
      String cameraName = entry.getKey();
      Transform3d robotToCamera = entry.getValue();
      CameraIO io;
      if (Robot.isSimulation()) {
        io = new CameraIOSim(cameraName, robotToCamera, m_poseSupplier);
      } else {
        io = new CameraIOHardware(cameraName, robotToCamera);
      }
      SingleInputPoseEstimator estimator = new SingleInputPoseEstimator(
          m_filter,
          m_networkLogger,
          io,
          this::addEstimate);
      m_estimators.add(estimator);
    }
  }

  private void updateEstimators() {
    m_loopTimer.reset();
    if (VisionConstants.kEnableMultiInputFilter || DriverStation.isDisabled()) {
      m_filter.clear();
    }
    for (SingleInputPoseEstimator estimator : m_estimators) {
      estimator.refresh(m_poseSupplier.get());
    }

    for (SingleInputPoseEstimator estimator : m_estimators) {
      estimator.run();
    }
    // finish logging
    m_logBuilder.log(m_poseSupplier.get());
    m_networkLogger.updateUnread();
    m_loopTimer.log();
  }

  public void startThread() {
    m_notifier.startPeriodic(VisionConstants.kPeriodic);
  }

  private void addEstimate(TimestampedPoseEstimate estimate) {
    if (VisionConstants.kEnableMultiInputFilter || DriverStation.isDisabled()) {
      if (!m_filter.verify(estimate.pose())) {
        return;
      }
    } 
    m_consumer.accept(estimate);
    m_networkLogger.registerValidEstimate(estimate.pose());
    m_logBuilder.addEstimate(estimate);
  }

  @Override
  public void close() {
    m_notifier.close();
  }
}
