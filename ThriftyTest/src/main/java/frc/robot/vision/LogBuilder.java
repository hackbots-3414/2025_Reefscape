package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class LogBuilder {
    private List<TimestampedPoseEstimate> m_estimates;
    private List<VisionLog> m_logs;
    private Pose2d m_result;

    public LogBuilder() {
        m_estimates = new ArrayList<>();
        m_logs = new ArrayList<>();
    }

    public void setResult(Pose2d result) {
        m_result = result;
    }

    public void addEstimate(TimestampedPoseEstimate estimate) {
        m_estimates.add(estimate);
    }

    private void buildLogs() {
        for (TimestampedPoseEstimate est : m_estimates) {
            double distance = est.pose().minus(m_result)
                .getTranslation()
                .getNorm();
            m_logs.add(
                new VisionLog(est, distance)
            );
        }
    }

    public void log() {
        buildLogs();
        VisionLogger.record(m_logs);
    }

    /* a helper record to handle logs */
    public record VisionLog(
        TimestampedPoseEstimate estimate,
        double error
    ) {}
}
