package frc.robot.utils;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.vision.localization.TimestampedPoseEstimate;

/**
 * Wraps multiple pose estimator instances for easier handling.
 */
public class PoseEstimators {
  private final SwerveDrivePoseEstimator globalEstimator;
  private final SwerveDrivePoseEstimator reefEstimator;
  private final SwerveDrivePoseEstimator odometryEstimator;

  private final OnboardLogger logger = new OnboardLogger("Poses");

  public PoseEstimators(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions) {
    globalEstimator =
        new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, Pose2d.kZero);
    reefEstimator =
        new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, Pose2d.kZero);
    odometryEstimator =
        new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, Pose2d.kZero);
    logger.registerPose("Pure Odometry", odometryEstimator::getEstimatedPosition);
    logger.registerPose("Full-field", globalEstimator::getEstimatedPosition);
    logger.registerPose("Reef only", reefEstimator::getEstimatedPosition);
  }

  /**
   * Updates the estimators with the gyro angle as well as the wheel positions.
   */
  public void update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    globalEstimator.update(gyroAngle, wheelPositions);
    reefEstimator.update(gyroAngle, wheelPositions);
    odometryEstimator.update(gyroAngle, wheelPositions);
    logger.log();
  }

  public void resetPose(Pose2d pose) {
    globalEstimator.resetPose(pose);
    reefEstimator.resetPose(pose);
    odometryEstimator.resetPose(pose);
  }

  public void resetRotation(Rotation2d rotation) {
    globalEstimator.resetRotation(rotation);
    reefEstimator.resetRotation(rotation);
    odometryEstimator.resetRotation(rotation);
  }

  public void addPoseEstimate(TimestampedPoseEstimate estimate) {
    globalEstimator.addVisionMeasurement(estimate.pose(), estimate.timestamp(), estimate.stdDevs());
    if (estimate.isReefOnly()) {
      reefEstimator.addVisionMeasurement(estimate.pose(), estimate.timestamp(), estimate.stdDevs());
    }
  }

  public Pose2d getOdometryPose() {
    return odometryEstimator.getEstimatedPosition();
  }

  public Pose2d getReefPose() {
    return reefEstimator.getEstimatedPosition();
  }

  public Pose2d getGlobalPose() {
    return globalEstimator.getEstimatedPosition();
  }
}
