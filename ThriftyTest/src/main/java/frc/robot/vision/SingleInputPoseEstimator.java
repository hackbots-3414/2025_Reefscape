package frc.robot.vision;

import java.util.ArrayList;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotObserver;
import frc.robot.vision.TimestampedPoseEstimate.EstimationAlgorithm;

public class SingleInputPoseEstimator implements Runnable {
  private final Logger m_logger = LoggerFactory.getLogger(SingleInputPoseEstimator.class);

  private final PhotonCamera m_camera;
  private final Consumer<TimestampedPoseEstimate> m_reporter;

  // Estimators
  private final PhotonPoseEstimator m_estimator;

  private final String m_name;

  private final Transform3d m_robotToCamera;

  public SingleInputPoseEstimator(
      PhotonCamera camera,
      Transform3d robotToCamera,
      Consumer<TimestampedPoseEstimate> updateCallback) {
    m_camera = camera;
    m_name = camera.getName();
    m_reporter = updateCallback;
    m_robotToCamera = robotToCamera;
    m_estimator = new PhotonPoseEstimator(
        VisionConstants.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCamera);
    m_estimator.setMultiTagFallbackStrategy(
        PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  @Override
  public void run() {
    if (!m_camera.isConnected()) {
      SmartDashboard.putBoolean("Vision/" + m_name + " Connected", false);
      m_logger.error("Unable to read data from {}", m_name);
    } else {
      SmartDashboard.putBoolean("Vision/" + m_name + " Connected", true);
    }
    // Pull the latest data from the camera.
    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
    m_estimator.addHeadingData(
        RobotController.getMeasureTime().in(Seconds),
        RobotObserver.getPose().getRotation());
    /* take many */
    for (PhotonPipelineResult result : results) {
      // headingHandleResult(result);
      // handleResult(result);
      combinedHandleResult(result);
    }
  }

  private void combinedHandleResult(PhotonPipelineResult result) {
    // some prechecks before we do anything
    if (!precheckValidity(result))
      return;
    // we can now assume that we have targets
    List<PhotonTrackedTarget> targets = result.getTargets();
    // use solvePnP every time
    EstimationAlgorithm algorithm = (targets.size() > 1) ? EstimationAlgorithm.PnP
        : EstimationAlgorithm.Trig;

    Optional<EstimatedRobotPose> est = m_estimator.update(result);
    if (est.isPresent()) {
      Optional<TimestampedPoseEstimate> processed =
          process(result, est.get().estimatedPose, algorithm);
      if (processed.isPresent()) {
        m_reporter.accept(processed.get());
        return;
      }
    }
    PhotonTrackedTarget target = targets.get(0);
    int fidId = target.getFiducialId();
    Optional<Pose3d> targetPosition = VisionConstants.kTagLayout
        .getTagPose(fidId);
    if (targetPosition.isEmpty()) {
      m_logger.error("Tag {} detected not in field layout", fidId);
      return;
    }

    Pose3d targetPosition3d = targetPosition.get();
    Transform3d best3d = target.getBestCameraToTarget();
    Transform3d alt3d = target.getAlternateCameraToTarget();
    Pose3d best = targetPosition3d
        .plus(best3d.inverse())
        .plus(m_robotToCamera.inverse());
    Pose3d alt = targetPosition3d
        .plus(alt3d.inverse())
        .plus(m_robotToCamera.inverse());
    double bestHeading = best.getRotation().getZ();
    double altHeading = alt.getRotation().getZ();
    Pose2d pose = RobotObserver.getPose();
    double heading = pose.getRotation().getRadians();
    Transform2d bestDiff = best.toPose2d().minus(pose);
    Transform2d altDiff = alt.toPose2d().minus(pose);
    double bestRotErr = Math.abs(bestHeading - heading);
    double altRotErr = Math.abs(altHeading - heading);
    double bestXYErr = bestDiff.getTranslation().getNorm();
    double altXYErr = altDiff.getTranslation().getNorm();
    Pose3d estimate;

    if (Math.abs(bestRotErr - altRotErr) >= VisionConstants.kHeadingThreshold) {
      estimate = (bestRotErr <= altRotErr) ? best : alt;
    } else {
      estimate = (bestXYErr <= altXYErr) ? best : alt;
    }

    process(result, estimate, EstimationAlgorithm.Heading).ifPresent(m_reporter);

  }

  private boolean precheckValidity(PhotonPipelineResult result) {
    double latency = result.metadata.getLatencyMillis() * 1e-3;
    // too old -> don't count it
    if (latency > VisionConstants.kLatencyThreshold) {
      // this is interesting, so let's report it
      m_logger.warn("({}) Refused old vision data, latency of {}", m_name, latency);
      return false;
    }
    // no targets -> no pose
    return result.hasTargets();
  }

  private Optional<TimestampedPoseEstimate> process(
      PhotonPipelineResult result,
      Pose3d pose,
      EstimationAlgorithm algorithm) {
    double latency = result.metadata.getLatencyMillis() / 1.0e+3;
    double timestamp = Utils.getCurrentTimeSeconds() - latency;
    double ambiguity = getAmbiguity(result);
    Pose2d flatPose = pose.toPose2d();
    Matrix<N3, N1> stdDevs = calculateStdDevs(result, flatPose);
    // if in reef mode, disregard data that doesn't use the reef.
    if (RobotObserver.getReefMode()) {
      int id = result.getBestTarget().getFiducialId();
      if (!VisionConstants.kReefIds.contains(id)) {
        return Optional.empty();
      }
    }
    // check validity again
    if (!checkValidity(pose, ambiguity))
      return Optional.empty();
    List<Integer> tags = new ArrayList<>();
    for (PhotonTrackedTarget target : result.getTargets()) {
      tags.add(target.getFiducialId());
    }
    return Optional.of(
        new TimestampedPoseEstimate(flatPose, m_name, timestamp, stdDevs, algorithm, tags));
  }

  private boolean checkValidity(
      Pose3d pose,
      double ambiguity) {
    if (ambiguity >= VisionConstants.kAmbiguityThreshold)
      return false;
    return !isOutsideField(pose);
  }

  private boolean isOutsideField(Pose3d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double z = pose.getZ();
    double xMax = VisionConstants.kXYMargin.magnitude()
        + FieldConstants.kFieldLength.magnitude();
    double yMax = VisionConstants.kXYMargin.magnitude()
        + FieldConstants.kFieldWidth.magnitude();
    double xyMin = -VisionConstants.kXYMargin.magnitude();
    double zMax = VisionConstants.kZMargin.magnitude();
    double zMin = -VisionConstants.kZMargin.magnitude();
    return x < xyMin
        || x > xMax
        || y < xyMin
        || y > yMax
        || z > zMax
        || z < zMin;
  }

  private Matrix<N3, N1> calculateStdDevs(PhotonPipelineResult result, Pose2d pose) {
    double latency = result.metadata.getLatencyMillis() * 1e-3;
    double multiplier = calculateStdDevMultiplier(result, latency, pose);
    return VisionConstants.kBaseStdDevs.times(multiplier);
  }

  private double calculateStdDevMultiplier(
      PhotonPipelineResult result,
      double latency,
      Pose2d pose) {
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
        VisionConstants.kDistanceMultiplier
            * (averageTagDistance - VisionConstants.kNoisyDistance));
    // calculate an (average) ambiguity real quick:
    double ambiguity = getAmbiguity(result);
    // ambiguity factor
    double ambiguityFactor = Math.max(1,
        VisionConstants.kAmbiguityMultiplier * ambiguity
            + VisionConstants.kAmbiguityShifter);
    // tag divisor
    double tags = result.getTargets().size();
    double tagDivisor = 1 + (tags - 1) * VisionConstants.kTargetMultiplier;
    // distance from last pose
    double poseDifferenceError = Math.max(0,
        RobotObserver.getPose().minus(pose).getTranslation().getNorm()
            - VisionConstants.kDifferenceThreshold * RobotObserver.getVelocity());
    double diffMultiplier = Math.max(1,
        poseDifferenceError * VisionConstants.kDifferenceMultiplier);
    double timeMultiplier = Math.max(1, latency * VisionConstants.kLatencyMultiplier);
    // final calculation
    double stdDevMultiplier = ambiguityFactor
        * distanceFactor
        * diffMultiplier
        * timeMultiplier
        / tagDivisor;
    return stdDevMultiplier;
  }

  private double getAmbiguity(PhotonPipelineResult result) {
    return result.getBestTarget().getPoseAmbiguity();
  }
}

