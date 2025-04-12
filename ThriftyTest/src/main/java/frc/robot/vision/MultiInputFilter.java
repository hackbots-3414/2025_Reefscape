package frc.robot.vision;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.Map.Entry;

import org.slf4j.Logger;

import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;

public class MultiInputFilter {
    private final Logger m_logger = LoggerFactory.getLogger(MultiInputFilter.class);

    private HashMap<String, Set<Integer>> m_tags = new HashMap<>();

    /**
     * Returns whether a camera at the source is able to "see" the tag with the
     * specified ID, using the known camera horizontal field of view.
     */
    private boolean verifyTarget(Pose2d source, int tag) {
        Optional<Pose3d> tagPose = VisionConstants.k_layout.getTagPose(tag);
        if (tagPose.isEmpty()) return false;
        Transform2d relative = tagPose.get().toPose2d().minus(source);
        double angleToTag = Math.abs(Math.atan2(relative.getY(), relative.getX()));
        boolean angleToTagOk = angleToTag < VisionConstants.kHorizontalFov.getRadians() / 2;
        double tagFacingAngle = Math.abs(relative.getRotation().getRotations());
        boolean tagFacingAngleOk = tagFacingAngle > 0.25;
        if (angleToTagOk && tagFacingAngleOk) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Returns whether all targets provided COULD be seen with this camera
     */
    private boolean verifyTargets(Pose2d source, Set<Integer> targets) {
        for (int target : targets) {
            if (!verifyTarget(source, target)) return false;
        }
        return true;
    }
    
    public void addEstimate(TimestampedPoseEstimate est) {
        if (!m_tags.containsKey(est.source())) {
            m_tags.put(est.source(), new HashSet<>());
        }
        Set<Integer> visible = m_tags.get(est.source());
        visible.addAll(est.tags());
    }

    public void clear() {
        m_tags.clear();
    }

    public boolean verify(Pose2d estimate) {
        for (Entry<String, Set<Integer>> entry : m_tags.entrySet()) {
            String sourceName = entry.getKey();
            Set<Integer> tags = entry.getValue();
            if (!VisionConstants.cameras.containsKey(sourceName)) {
                m_logger.warn("Detected target not on field layout, ignoring");
                continue;
            }
            Transform3d offset = VisionConstants.cameras.get(sourceName);
            Transform2d offset2d = new Transform2d(
                offset.getX(),
                offset.getY(),
                offset.getRotation().toRotation2d()
            );
            Pose2d source = estimate.plus(offset2d);
            if (!verifyTargets(source, tags)) return false;
        }
        // no checks have failed, i.e. all checks have passed.
        return true;
    }

}
