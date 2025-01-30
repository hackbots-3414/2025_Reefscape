package frc.robot.vision;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class SingleTagEstimator {
    private Logger m_logger = LoggerFactory.getLogger(SingleTagEstimator.class);
    
    private AprilTagFieldLayout m_layout;
    private Transform3d m_robotToCamera;

    public SingleTagEstimator(
        AprilTagFieldLayout layout,
        Transform3d robotToCamera
    ) {
        m_layout = layout;
        m_robotToCamera = robotToCamera;
    }

    public Optional<Pose3d> estimate(
        PhotonTrackedTarget target
    ) {
        int id = target.getFiducialId();
        Optional<Pose3d> tagPositionMaybe = m_layout.getTagPose(id);
        if (tagPositionMaybe.isEmpty()) {
            m_logger.warn("I found a tag that doesn't exist: {}", id);
            return Optional.empty();
        };

        Pose3d tagPosition = tagPositionMaybe.get();
        Transform3d cameraToTarget = calculateCameraToTarget(target);

        Pose3d cameraPosition = tagPosition.plus(cameraToTarget.inverse());

        Pose3d robotPosition = cameraPosition.plus(
            m_robotToCamera.inverse()
        );

        return Optional.of(
            robotPosition
        );
    }

    private Transform3d calculateCameraToTarget(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget();
    }
}
