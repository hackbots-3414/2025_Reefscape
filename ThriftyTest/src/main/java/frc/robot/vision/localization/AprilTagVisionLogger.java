package frc.robot.vision.localization;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.OnboardLogger;

public class AprilTagVisionLogger {
  private List<Pose2d> m_estimates;

  private final OnboardLogger m_logger;

  public AprilTagVisionLogger() {
    m_estimates = new ArrayList<>(20);
    m_logger = new OnboardLogger("Vision");
    m_logger.registerPoses("Estimates", () -> {
      Pose2d[] estimates = new Pose2d[m_estimates.size()];
      for (int i = 0; i < m_estimates.size(); i++) {
        estimates[i] = m_estimates.get(i);
      }
      return estimates;
    });
  }

  public void addEstimate(TimestampedPoseEstimate estimate) {
    if (LocalizationConstants.kEnableLogging) {
      m_estimates.add(estimate.pose());
    }
  }

  public void log() {
    if (LocalizationConstants.kEnableLogging) {
      m_logger.log();
      m_estimates.clear();
    }
  }
}
