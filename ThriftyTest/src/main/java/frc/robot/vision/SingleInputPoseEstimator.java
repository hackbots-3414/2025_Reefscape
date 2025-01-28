package frc.robot.vision;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class SingleInputPoseEstimator implements Runnable {
    private Logger m_logger = LoggerFactory.getLogger(SingleInputPoseEstimator.class);

    private PhotonCamera m_camera;
    private Consumer<TimestampedPoseEstimate> m_reporter;

    // Estimators
    private PhotonPoseEstimator m_estimator;
    private SingleTagEstimator m_singleTagEstimator;

    private Supplier<Optional<Integer>> m_singleTag;

    public SingleInputPoseEstimator(
        PhotonCamera camera,
        Transform3d robotToCamera,
        Consumer<TimestampedPoseEstimate> updateCallback,
        Supplier<Optional<Integer>> singleTagSupplier
    ) {
        m_camera = camera;
        m_reporter = updateCallback;
        m_singleTag = singleTagSupplier;
        AprilTagFieldLayout layout = null;
        try {
            layout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2025Reefscape.m_resourceFile
            );
        } catch (IOException e) {
            m_logger.error("failed to load resource file: {}", e);
            System.exit(1);
            // no need to worry about null exceptions if we just terminate now!
        }
        m_estimator = new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        );
        m_estimator.setMultiTagFallbackStrategy(
            PoseStrategy.LOWEST_AMBIGUITY
        );
        m_singleTagEstimator = new SingleTagEstimator(
            layout,
            robotToCamera
        );
    }

    public void run() {
        // Pull the latest data from the camera.
        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
        if (results.size() > VisionConstants.k_maxResults) {
            /*
            Rationale for this warning:
            This run() method should be running on a loop. It should run fast.
            Ideally, it runs WAY faster than the camera always receives either
            0 or 1 new result.
            We may want to know if we are being bombarded with too many results,
            i.e. the camera is running faster than we are, which could suggest
            that we are running slow.
            Also, we assume that the time that we see the result minus the time
            the result took to get sent to us is the time that it was sent.
            But if we are running slowly, it's possible there would be some
            time between when a result was sent and when we "see" it. This would
            mess up the timestamping logic.
            */
            m_logger.warn("Possibly too many result: {}", results.size());
        }
        for (PhotonPipelineResult result : results) {
            handleResult(result);
        }
    }

    private void handleResult(PhotonPipelineResult result) {
        boolean isValid = precheckValidity(result);
        if (!isValid) return;
        // By this point the result is valid.
        m_singleTag.get().ifPresentOrElse(
            (Integer id) -> {
                singleTag(result, id.intValue());
            },
            () -> {
                multiTag(result);
            }
        );
    }

    private boolean precheckValidity(PhotonPipelineResult result) {
        double latency = result.metadata.getLatencyMillis() / 1.0e+3;
        // too old -> don't count it
        if (latency > VisionConstants.k_latencyThreshold) {
            // this is interesting, so let's report it
            m_logger.warn("Refused old vision data, latency of {}", latency);
            return false;
        }
        // no targets -> no pose
        return result.hasTargets();
    }

    private void singleTag(
        PhotonPipelineResult result,
        int tagId
    ) {
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() != tagId) continue;
            // if we are running this, then we have the tag.
            m_singleTagEstimator.estimate(target).ifPresent((Pose3d est) -> {
                process(result, est).ifPresent(m_reporter);
            });
        }
    }

    private void multiTag(
        PhotonPipelineResult result
    ) {
        m_estimator.update(result).ifPresent((EstimatedRobotPose est) -> {
            process(result, est).ifPresent(m_reporter);
        });
    }
    
    private Optional<TimestampedPoseEstimate> process(
        PhotonPipelineResult result,
        EstimatedRobotPose estimation
    ) {
        double timestamp = Utils.getCurrentTimeSeconds()
            - result.metadata.getLatencyMillis() / 1.0e+3;
        Pose3d pose = estimation.estimatedPose;
        double ambiguity = getAmbiguity(result);
        Matrix<N3, N1> stdDevs = calculateStdDevs(result, timestamp);
        // check validity again
        if (!checkValidity(pose, ambiguity)) return Optional.empty();
        return Optional.of(
            new TimestampedPoseEstimate(pose.toPose2d(), timestamp, stdDevs)
        );
    }

    private Optional<TimestampedPoseEstimate> process(
        PhotonPipelineResult result,
        Pose3d estimate
    ) {
        double timestamp = Utils.getCurrentTimeSeconds()
            - result.metadata.getLatencyMillis() / 1.0e+3;
        Matrix<N3, N1> stdDevs = calculateStdDevs(result, timestamp);
        // check validity again, but set ambiguity to 0 so that we have no issues.
        if (!checkValidity(estimate, 0.0)) return Optional.empty();
        return Optional.of(
            new TimestampedPoseEstimate(estimate.toPose2d(), timestamp, stdDevs)
        );
    }

    private boolean checkValidity(
        Pose3d pose,
        double ambiguity
    ) {
        if (ambiguity >= VisionConstants.k_AmbiguityThreshold) return false;
        return !isOutsideField(pose);
    }
    
    private boolean isOutsideField(Pose3d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double z = pose.getZ();
        double xMax = VisionConstants.k_XYMargin.magnitude()
            + FieldConstants.k_fieldLength.magnitude();
        double yMax = VisionConstants.k_XYMargin.magnitude()
            + FieldConstants.k_fieldWidth.magnitude();
        double xyMin = -VisionConstants.k_XYMargin.magnitude();
        double zMax = VisionConstants.k_ZMargin.magnitude();
        double zMin = -VisionConstants.k_ZMargin.magnitude();
        return x < xyMin
            || x > xMax
            || y < xyMin
            || y > yMax
            || z > zMax
            || z < zMin;
    }

    private Matrix<N3, N1> calculateStdDevs(
        PhotonPipelineResult result,
        double latency
    ) {
        double multiplier = calculateStdDevMultiplier(result, latency);
        Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, Math.PI);
        return stdDevs.times(multiplier);
    }

    private double calculateStdDevMultiplier(
        PhotonPipelineResult result,
        double latency
    ) {
        double averageTagDistance = 0;
        for (PhotonTrackedTarget tag : result.getTargets()) {
            averageTagDistance += tag
                .getBestCameraToTarget()
                .getTranslation()
                .getNorm();
        }
        averageTagDistance /= result.getTargets().size();
        // calculate tag distance factor
        double distanceFactor = Math.max(1,
            VisionConstants.k_distanceMultiplier
                * (averageTagDistance - VisionConstants.k_noisyDistance)
        );
        // calculate an (average) ambiguity real quick:
        double ambiguity = getAmbiguity(result);
        // ambiguity factor
        double ambiguityFactor = Math.max(1,
            VisionConstants.k_ambiguityMultiplier * ambiguity
                + VisionConstants.k_ambiguityShifter
        );
        // tag divisor
        double tags = result.getTargets().size();
        double tagDivisor = 1 + (tags - 1) * VisionConstants.k_targetMultiplier;
        // final calculation
        double stdDevMultiplier = ambiguityFactor * distanceFactor / tagDivisor;
        return stdDevMultiplier;
    }

    private double getAmbiguity(PhotonPipelineResult result) {
        return result.getBestTarget().getPoseAmbiguity();
    }
}