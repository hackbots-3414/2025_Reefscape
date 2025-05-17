package frc.robot.vision;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotObserver;

public class VisionNetworkLogger {
  private final boolean m_enableLogging;

  private final ArrayList<Pose2d> m_estimates;
  private final ArrayList<Pose2d> m_rejected;

  public VisionNetworkLogger(boolean enable) {
    m_enableLogging = enable;
    m_estimates = new ArrayList<>();
    m_rejected = new ArrayList<>();
  }

  public void registerValidEstimate(TimestampedPoseEstimate estimate) {
    m_estimates.add(estimate.pose());
  }

  public void registerRejectedEstimate(TimestampedPoseEstimate estimate) {
    m_rejected.add(estimate.pose());
  }

  public void update() {
    if (m_enableLogging) {
      RobotObserver.getField().getObject(VisionConstants.kEstimationName).setPoses(m_estimates);
      RobotObserver.getField().getObject(VisionConstants.kRejectedName).setPoses(m_rejected);
      double estimates = m_estimates.size();
      double rejected = m_rejected.size();
      SmartDashboard.putNumber("Vision/Estimates", estimates);
      SmartDashboard.putNumber("Vision/Rejected", rejected);
      if (estimates > 0) {
        SmartDashboard.putNumber("Rejection rate", rejected / estimates);
      } else {
        SmartDashboard.putNumber("Rejection rate", Double.NaN);
      }
    }
    // clean up after ourselves
    m_estimates.clear();
    m_rejected.clear();
  }
}
