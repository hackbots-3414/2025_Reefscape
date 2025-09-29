package frc.robot.vision.tracking;

import java.util.ArrayDeque;
import java.util.Optional;
import java.util.Queue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.vision.tracking.AlgaeTracker.ObjectTrackingStatus;

public class SimplePoseFilter {
  private record PoseData(Translation3d pos, double trust) {
  };

  private final Queue<ObjectTrackingStatus> m_data;
  private static final int SIZE_LIMIT = 100;

  public SimplePoseFilter() {
    m_data = new ArrayDeque<>(SIZE_LIMIT);
  }

  public void add(ObjectTrackingStatus status) {
    // Add status to list
    m_data.add(status);
    while (m_data.size() > SIZE_LIMIT) {
      m_data.remove();
    }
  }

  public Optional<Pose3d> calculate() {
    // Ensure that list is below size limit.
    Optional<PoseData> result = m_data.stream()
        .filter(ObjectTrackingStatus::isOkay)
        .filter(s -> s.pose().isPresent())
        .map(s -> {
          double trust = calculateTrust(s);
          return new PoseData(
              s.pose().get().getTranslation().times(trust),
              trust);
        })
        .reduce((a, b) -> new PoseData(a.pos().plus(b.pos()), a.trust() + b.trust()));
    // If we have nothing, say so.
    if (result.isEmpty()) {
      return Optional.empty();
    }
    PoseData data = result.get();
    Pose3d combined = new Pose3d(data.pos().div(data.trust()), Rotation3d.kZero);
    return Optional.of(combined);
  }

  private double calculateTrust(ObjectTrackingStatus status) {
    double dt = Timer.getTimestamp() - status.time();
    if (dt == 0) {
      return 1;
    }
    return 1 / dt;
  }
}
