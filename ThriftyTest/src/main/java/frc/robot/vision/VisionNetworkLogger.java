package frc.robot.vision;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotObserver;

public class VisionNetworkLogger {
  private final boolean m_enableLogging;

  private final ArrayList<Pose2d> m_estimates;
  private final ArrayList<Pose2d> m_rejects;

  public VisionNetworkLogger(boolean enable) {
    m_enableLogging = enable;
    m_estimates = new ArrayList<>();
    m_rejects = new ArrayList<>();;
  }

  public void registerValidEstimate(Pose2d estimate) {
    if (m_enableLogging) {
      m_estimates.add(estimate);
    }
  }

  public void registerRejectedEstimate(Pose2d estimate) {
    if (m_enableLogging) {
      m_rejects.add(estimate);
    }
  }

  public void updateUnread() {
    if (m_enableLogging) {
      RobotObserver.getField().getObject(VisionConstants.kEstimationName).setPoses(m_estimates);
      RobotObserver.getField().getObject(VisionConstants.kRejectName).setPoses(m_rejects);
      double estimates = m_estimates.size();
      double rejects = m_rejects.size();
      SmartDashboard.putNumber("Vision/Estimate Count", estimates);
      SmartDashboard.putNumber("Vision/Reject Count", rejects);
      if (rejects + estimates > 0) {
        SmartDashboard.putNumber("Vision/Rejection rate", rejects / (estimates + rejects));
      } else {
        SmartDashboard.putNumber("Vision/Rejection rate", Double.NaN);
      }
    }
    // clean up after ourselves
    m_estimates.clear();
    m_rejects.clear();
  }
}
