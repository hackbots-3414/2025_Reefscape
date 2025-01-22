package frc.robot.vision;

import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.lang.annotation.Target;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import ch.qos.logback.core.Layout;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SingleInputPoseEstimator implements Runnable {
    private Logger m_logger = LoggerFactory.getLogger(SingleInputPoseEstimator.class);

    private PhotonCamera m_camera;
    private PhotonPoseEstimator m_estimator;
    private Consumer<TimestampedPoseEstimate> m_reporter;

    public SingleInputPoseEstimator(
        PhotonCamera camera,
        Transform3d robotToCamera,
        Consumer<TimestampedPoseEstimate> callback
    ) {
        m_camera = camera;
        m_reporter = callback;
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
    }
    
    public SingleInputPoseEstimator(
        PhotonCamera camera,
        Transform3d robotToCamera,
        CommandSwerveDrivetrain drivetrain
    ) {
        this(camera, robotToCamera, drivetrain::addPoseEstimate);
    }

    public void run() {
        // Pull the latest data from the camera.
        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            boolean isValid = checkValidity(
                result,
                result.metadata.getLatencyMillis() / 1.0e3
            );
            if (isValid) {
                // report the valid estimate to our estimator
                m_estimator.update(result).ifPresent((EstimatedRobotPose estimation) -> {
                    process(result, estimation);
                });
            };
        }
    }

    private void process(
        PhotonPipelineResult result,
        EstimatedRobotPose estimation
    ) {
        double timestamp = result.timestampSeconds;
        // please be reasonable values.
        m_logger.info("timestamp is {}", timestamp);
    }
    
    private boolean checkValidity(PhotonPipelineResult result, double latency) {
        // too old -> don't count it
        if (latency > VisionConstants.k_latencyThreshold) {
            // this is interesting, so let's report it
            m_logger.warn("Refused old vision data, latency of {}", latency);
            return false;
        }
        // no targets -> no pose
        if (!result.hasTargets()) return false;
        // pull the data from the result
        Pose3d pose = getPose(result);
        double ambiguity = getAmbiguity(result);
        // check for ambiguity
        if (ambiguity > VisionConstants.k_AmbiguityThreshold) return false;
        if (isOutsideField(pose)) return false;
        return true;
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
        // ambiguity factor
        double ambiguity = getAmbiguity(result);
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
}