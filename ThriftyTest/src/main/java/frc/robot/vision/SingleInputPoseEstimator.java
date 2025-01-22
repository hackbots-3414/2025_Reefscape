package frc.robot.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SingleInputPoseEstimator implements Runnable {
    private PhotonCamera m_camera;
    // this should be defined such that this is FROM camera TO robot.
    // i.e. m_transform = robot - camera
    // so  robot = camera + transform
    // and camera = robot - transform
    private Transform3d m_transform;
    private Consumer<TimestampedPoseEstimate> m_reporter;
    
    public SingleInputPoseEstimator(
        PhotonCamera camera,
        Transform3d robotToCamera,
        CommandSwerveDrivetrain drivetrain
    ) {
        m_camera = camera;
        m_reporter = drivetrain::addPoseEstimate;
        m_transform = robotToCamera.inverse();
        // robotToCamera is  camera - robot
        // so its inverse is robot - camera
        // which is cameraToRobot.
    }

    public void run() {
        // Pull the latest data from the camera.
        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            Optional<TimestampedPoseEstimate> processed = process(result);
            processed.ifPresent((estimate) -> {
                // report the valid estimate
                m_reporter.accept(estimate);
            });
        }
    }

    private Optional<TimestampedPoseEstimate> process(
        PhotonPipelineResult result
    ) {
        /*
        SUPER IMPORTANT:
        The following logic makes the assumption that this runnable is running
        fast enough such that results don't pile up in the queue.
        This is important, because it allows us to make the assumption that
        the time that we read the result minus the latency to post it is equal
        to the time that the result was sent.
         */
        double latency = result.metadata.getLatencyMillis() / 1e3;
        double timestamp = Utils.getCurrentTimeSeconds()
            - latency;
        if (!checkValidity(result, timestamp)) {
            return Optional.empty();
        }
        Transform3d cameraPosition = result
            .getMultiTagResult()
            .get()
            .estimatedPose
            .best;
        Pose2d robotPosition = new Pose3d()
            .plus(cameraPosition)
            .plus(m_transform)
            .toPose2d();
        Matrix<N3, N1> stdDevs = calculateStdDevs(result, latency);
        TimestampedPoseEstimate estimate = new TimestampedPoseEstimate(
            robotPosition,
            timestamp,
            stdDevs);
        return Optional.of(estimate);
    }

    private boolean checkValidity(PhotonPipelineResult result, double latency) {
        // too old -> don't count it
        if (latency > VisionConstants.k_latencyThreshold) return false;
        // no targets -> no pose
        if (!result.hasTargets()) return false;
        // If for some reason we can't do this
        Optional<MultiTargetPNPResult> multitag = result.getMultiTagResult();
        if (multitag.isEmpty()) return false;
        // get the result
        PnpResult estimation = multitag.get().estimatedPose;
        if (estimation.ambiguity > VisionConstants.k_AmbiguityThreshold) {
            // pose ambiguity is too high.
            return false;
        }
        Pose3d pose = new Pose3d().plus(estimation.best).plus(m_transform);
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
        double ambiguity = result
            .getMultiTagResult()
            .get()
            .estimatedPose
            .ambiguity;
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