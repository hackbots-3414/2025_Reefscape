package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotObserver;

public class LogBuilder {
    private List<TimestampedPoseEstimate> m_estimates;
    private List<VisionLog> m_logs;

    public LogBuilder() {
        m_estimates = new ArrayList<>(20);
        m_logs = new ArrayList<>(20);
    }

    public void addEstimate(TimestampedPoseEstimate estimate) {
        m_estimates.add(estimate);
    }

    private void buildLogs() {
        m_logs.clear();
        for (TimestampedPoseEstimate est : m_estimates) {
            Pose2d robot = RobotObserver.getPose();
            double distance = est.pose().minus(robot)
                .getTranslation()
                .getNorm();
            m_logs.add(
                new VisionLog(est, distance, robot)
            );
        }
        // consume each processed value
        m_estimates.clear();
    }
    

    public void log() {
        buildLogs();
        if (VisionConstants.k_enableLogging) VisionLogger.record(m_logs);
    }

    /* a helper record to handle logs */
    public record VisionLog(
        TimestampedPoseEstimate estimate,
        double error,
        Pose2d robot
    ) {}
}
