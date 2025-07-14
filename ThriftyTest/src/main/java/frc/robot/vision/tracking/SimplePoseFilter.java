package frc.robot.vision.tracking;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.vision.tracking.AlgaeTracker.ObjectTrackingStatus;

public class SimplePoseFilter {
    private record PoseData(ObjectTrackingStatus status, double trust) {};

    private final Set<PoseData> m_data;

    public SimplePoseFilter() {
        m_data = new HashSet<>();
    }

    public Optional<Pose2d> calculate(ObjectTrackingStatus status) {
        // Add new value with calculated trust
        m_data.add(new PoseData(status, calculateTrust(status)));
        // Filter old values from m_data
        filterData();
        // If no data, return Optional.empty()
        return weighAll();
    }

    private double calculateTrust(ObjectTrackingStatus status) {
        double timeDiff = Timer.getTimestamp() - status.time();
        if (timeDiff == 0) {
            return 1;
        }
        return Math.min(1, 1/timeDiff);
    }
}