package frc.robot.vision;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotObserver;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.vision.TimestampedPoseEstimate.EstimationAlgorithm;

public class SingleInputPoseEstimator implements Runnable {
    private Logger m_logger = LoggerFactory.getLogger(SingleInputPoseEstimator.class);

    private PhotonCamera m_camera;
    private Consumer<TimestampedPoseEstimate> m_reporter;

    private long m_lastUpdate;

    // Estimators
    private PhotonPoseEstimator m_pnpEstimator;
    private PhotonPoseEstimator m_trigEstimator;

    private String m_name;

    public SingleInputPoseEstimator(
        PhotonCamera camera,
        Transform3d robotToCamera,
        Consumer<TimestampedPoseEstimate> updateCallback
    ) {
        m_camera = camera;
        m_name = camera.getName();
        m_reporter = updateCallback;
        m_pnpEstimator = new PhotonPoseEstimator(
            VisionConstants.k_layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        );
        m_pnpEstimator.setMultiTagFallbackStrategy(
            PoseStrategy.LOWEST_AMBIGUITY
        );
        m_trigEstimator = new PhotonPoseEstimator(
            VisionConstants.k_layout,
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            robotToCamera
        );
        m_lastUpdate = System.nanoTime();
    }

    @Override
    public void run() {
        long newTime = System.nanoTime();
        double dt = (double)(newTime - m_lastUpdate) / 1.0e9;
        m_logger.info("dt: {}", dt);
        // Pull the latest data from the camera.
        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
        if (results.size() > VisionConstants.k_fps * dt) {
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
            m_logger.warn("Possibly too many results: {} ({})", results.size(), m_camera.getName());
        }
        /* take many */
        for (PhotonPipelineResult result : results) {
            handleResult(result);
        }
        m_lastUpdate = newTime;
    }

    private void handleResult(PhotonPipelineResult result) {
        boolean isValid = precheckValidity(result);
        if (!isValid) return;
        // By this point the result is valid.
        PhotonPoseEstimator estimator;
        if (RobotObserver.getReefMode()) {
            // Update heading data
            m_trigEstimator.addHeadingData(
                RobotController.getMeasureTime().in(Seconds),
                RobotObserver.getPose().getRotation()
            );
            estimator = m_trigEstimator;
        } else {
            estimator = m_pnpEstimator;
        }
        estimator.update(result).ifPresent((est) -> {
            process(result, est).ifPresent(m_reporter);
        });
    }

    private boolean precheckValidity(PhotonPipelineResult result) {
        double latency = result.metadata.getLatencyMillis() / 1.0e+3;
        // too old -> don't count it
        if (latency > VisionConstants.k_latencyThreshold) {
            // this is interesting, so let's report it
            m_logger.warn("({}) Refused old vision data, latency of {}", m_name, latency);
            return false;
        }
        // no targets -> no pose
        return result.hasTargets();
    }

    private Optional<TimestampedPoseEstimate> process(
        PhotonPipelineResult result,
        EstimatedRobotPose estimation
    ) {
        double latency = result.metadata.getLatencyMillis() / 1.0e+3;
        double timestamp = Utils.getCurrentTimeSeconds() - latency;
        Pose3d pose = estimation.estimatedPose;
        double ambiguity = getAmbiguity(result);
        Pose2d flatPose = pose.toPose2d();
        Matrix<N3, N1> stdDevs = calculateStdDevs(result, latency, flatPose);
        // if in reef mode, disregard data that doesn't use the reef.
        if (RobotObserver.getReefMode()) {
            int id = result.getBestTarget().getFiducialId();
            if (!VisionConstants.k_reefIds.contains(id)) {
                return Optional.empty();
            }
        }
        // check validity again
        if (!checkValidity(pose, ambiguity)) return Optional.empty();
        EstimationAlgorithm algorithm = (RobotObserver.getReefMode()) ?
            EstimationAlgorithm.Trig
            :
            EstimationAlgorithm.PnP;
        return Optional.of(
            new TimestampedPoseEstimate(flatPose, m_name, timestamp, stdDevs, algorithm)
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
        double latency,
        Pose2d pose
    ) {
        double multiplier = calculateStdDevMultiplier(result, latency, pose);
        Matrix<N3, N1> stdDevs = VecBuilder.fill(1.0, 1.0, Math.PI);
        return stdDevs.times(multiplier);
    }

    private double calculateStdDevMultiplier(
        PhotonPipelineResult result,
        double latency,
        Pose2d pose
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
        // distance from last pose
        double poseDifferenceError = Math.max(0,
            RobotObserver.getPose().minus(pose).getTranslation().getNorm()
                - VisionConstants.k_differenceThreshold * RobotObserver.getVelocity()
        );
        double diffMultiplier = Math.max(1,
            poseDifferenceError * VisionConstants.k_differenceMultiplier
        );
        // final calculation
        double stdDevMultiplier = ambiguityFactor
            * distanceFactor
            * diffMultiplier
            / tagDivisor;
        return stdDevMultiplier;
    }

    private double getAmbiguity(PhotonPipelineResult result) {
        return result.getBestTarget().getPoseAmbiguity();
    }
}
