package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotObserver;

public class VisionHandler implements AutoCloseable {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(VisionHandler.class);

  private final Consumer<TimestampedPoseEstimate> m_consumer;

  private final Notifier m_notifier;
  private final List<SingleInputPoseEstimator> m_estimators = new ArrayList<>();

  private final VisionLogBuilder m_logBuilder;

  private final MultiInputFilter m_filter;

  private final Alert m_badAlert = new Alert("Test?", "bad", AlertType.kError);

  public VisionHandler(Consumer<TimestampedPoseEstimate> callback) {
    m_consumer = callback;
    m_filter = new MultiInputFilter();
    setupCameras();
    m_notifier = new Notifier(this::updateEstimators);
    m_logBuilder = new VisionLogBuilder();
  }

  private void setupCameras() {
    for (Map.Entry<String, Transform3d> entry : VisionConstants.kCameras.entrySet()) {
      String cameraName = entry.getKey();
      Transform3d robotToCamera = entry.getValue();
      CameraIO io;
      if (Robot.isSimulation()) {
        io = new CameraIOSim(cameraName, robotToCamera);
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
    m_filter.clear();
    long start = System.currentTimeMillis();
    for (SingleInputPoseEstimator estimator : m_estimators) {
      estimator.refresh();
    }

    for (SingleInputPoseEstimator estimator : m_estimators) {
      estimator.run();
    }
    long elapsed = System.currentTimeMillis() - start;
    SmartDashboard.putNumber("Vision/Loop time (ms)", elapsed);

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
