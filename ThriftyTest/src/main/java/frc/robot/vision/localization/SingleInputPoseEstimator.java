package frc.robot.vision.localization;

import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotObserver;
import frc.robot.vision.CameraIO;
import frc.robot.vision.CameraIO.CameraIOInputs;

public class SingleInputPoseEstimator implements Runnable {
  private final Logger m_logger = LoggerFactory.getLogger(SingleInputPoseEstimator.class);

  private final CameraIO m_io;
  private final CameraIOInputs m_inputs;

  private final Consumer<TimestampedPoseEstimate> m_reporter;
  private Pose2d m_lastPose;

  private final PhotonPoseEstimator m_estimator;
  private final MultiInputFilter m_filter;

  private final Alert m_disconnectedAlert;

  public SingleInputPoseEstimator(
      MultiInputFilter fitler,
      CameraIO io,
      Consumer<TimestampedPoseEstimate> updateCallback) {
    m_io = io;
    m_inputs = new CameraIOInputs();
    m_reporter = updateCallback;
    m_filter = fitler;
    m_disconnectedAlert = new Alert("Vision/Camera Status", io.getName() + " disconnected", AlertType.kError);
    m_estimator = new PhotonPoseEstimator(
        AprilTagVisionConstants.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        m_io.getRobotToCamera());
    m_estimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  public void refresh(Pose2d robotPose) {
    m_lastPose = robotPose;
    m_io.updateInputs(m_inputs);
    m_disconnectedAlert.set(!m_inputs.connected);
    if (!m_inputs.connected) {
      return;
    }
    for (PhotonPipelineResult result : m_inputs.unreadResults) {
      Set<Integer> tags = result.getTargets().stream()
          .map(target -> target.getFiducialId())
          .collect(Collectors.toSet());
      m_filter.addInput(m_io.getName(), tags);
    }
  }

  @Override
  public void run() {
    if (!m_inputs.connected) {
      return;
    }
    // Pull the latest data from the camera.
    List<PhotonPipelineResult> results = m_inputs.unreadResults;
    m_estimator.addHeadingData(
        RobotController.getMeasureTime().in(Seconds),
        m_lastPose.getRotation());
    /* take many */
    for (PhotonPipelineResult result : results) {
      combinedHandleResult(result);
    }
  }

  private void combinedHandleResult(PhotonPipelineResult result) {
    // some prechecks before we do anything
    if (!precheckValidity(result)) {
      return;
    }
    // we can now assume that we have targets
    List<PhotonTrackedTarget> targets = result.getTargets();
    // use solvePnP every time if we can
    Optional<EstimatedRobotPose> est = m_estimator.update(result);
    if (est.isPresent()) {
      Pose3d estimatedPose = est.get().estimatedPose;
      process(result, estimatedPose).ifPresent(m_reporter);
    }
    PhotonTrackedTarget target = targets.get(0);
    int fidId = target.getFiducialId();
    Optional<Pose3d> targetPosition = AprilTagVisionConstants.kTagLayout
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
        .plus(m_io.getRobotToCamera().inverse());
    Pose3d alt = targetPosition3d
        .plus(alt3d.inverse())
        .plus(m_io.getRobotToCamera().inverse());
    // final decision maker
    double bestHeading = best.getRotation().getZ();
    double altHeading = alt.getRotation().getZ();
    Pose2d pose = m_lastPose;
    double heading = pose.getRotation().getRadians();
    Transform2d bestDiff = best.toPose2d().minus(pose);
    Transform2d altDiff = alt.toPose2d().minus(pose);
    double bestRotErr = Math.abs(bestHeading - heading);
    double altRotErr = Math.abs(altHeading - heading);
    double bestXYErr = bestDiff.getTranslation().getNorm();
    double altXYErr = altDiff.getTranslation().getNorm();
    Pose3d estimate;

    if (Math.abs(bestRotErr - altRotErr) >= AprilTagVisionConstants.kHeadingThreshold) {
      estimate = (bestRotErr <= altRotErr) ? best : alt;
    } else {
      estimate = (bestXYErr <= altXYErr) ? best : alt;
    }

    process(result, estimate).ifPresent(m_reporter);
  }

  private boolean precheckValidity(PhotonPipelineResult result) {
    double latency = result.metadata.getLatencyMillis() * 1e-3;
    if (latency > AprilTagVisionConstants.kLatencyThreshold) {
      m_logger.warn("({}) Refused old vision data, latency of {}", m_io.getName(), latency);
      return false;
    }
    return result.hasTargets();
  }

  private Optional<TimestampedPoseEstimate> process(PhotonPipelineResult result, Pose3d pose) {
    double latency = result.metadata.getLatencyMillis() / 1.0e+3;
    double timestamp = Utils.getCurrentTimeSeconds() - latency;
    double ambiguity = getAmbiguity(result);
    Pose2d flatPose = pose.toPose2d();
    Matrix<N3, N1> stdDevs = calculateStdDevs(result, flatPose);

    // check validity
    if (!checkValidity(pose, ambiguity)) {
      return Optional.empty();
    }
    return Optional.of(
        new TimestampedPoseEstimate(flatPose, timestamp, stdDevs));
  }

  private boolean checkValidity(
      Pose3d pose,
      double ambiguity) {
    if (ambiguity >= AprilTagVisionConstants.kAmbiguityThreshold) {
      return false;
    }
    return !isOutsideField(pose);
  }

  private boolean isOutsideField(Pose3d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double z = pose.getZ();
    double xMax = AprilTagVisionConstants.kXYMargin.magnitude()
        + FieldConstants.kFieldLength.magnitude();
    double yMax = AprilTagVisionConstants.kXYMargin.magnitude()
        + FieldConstants.kFieldWidth.magnitude();
    double xyMin = -AprilTagVisionConstants.kXYMargin.magnitude();
    double zMax = AprilTagVisionConstants.kZMargin.magnitude();
    double zMin = -AprilTagVisionConstants.kZMargin.magnitude();
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
    return AprilTagVisionConstants.kBaseStdDevs.times(multiplier);
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
        AprilTagVisionConstants.kDistanceMultiplier
            * (averageTagDistance - AprilTagVisionConstants.kNoisyDistance));
    // calculate an (average) ambiguity real quick:
    double ambiguity = getAmbiguity(result);
    // ambiguity factor
    double ambiguityFactor = Math.max(1,
        AprilTagVisionConstants.kAmbiguityMultiplier * ambiguity
            + AprilTagVisionConstants.kAmbiguityShifter);
    // tag divisor
    double tags = result.getTargets().size();
    double tagDivisor = 1 + (tags - 1) * AprilTagVisionConstants.kTargetMultiplier;
    // distance from last pose
    double poseDifferenceError = Math.max(0,
        m_lastPose.minus(pose).getTranslation().getNorm()
            - AprilTagVisionConstants.kDifferenceThreshold * RobotObserver.getVelocity());
    double diffMultiplier = Math.max(1,
        poseDifferenceError * AprilTagVisionConstants.kDifferenceMultiplier);
    double timeMultiplier = Math.max(1, latency * AprilTagVisionConstants.kLatencyMultiplier);
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

  public boolean isConnected() {
    return m_inputs.connected;
  }
}

