package frc.robot.driveassist;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Autopilot is a class that tries to drive a target to a goal in 2 dimensional space.
 *
 * Autopilot is a fast algorithm because it doesn not think ahead. Any and all math is already
 * worked out such that a small amount of computation is necessary on the fly.
 *
 * This means that autopilot is un able to avoid obstacles, because it cannot think ahead.
 *
 * It also is not guarunteed to provide the fastest path - however, it is well tuned such that it
 * gives very close results.
 */
public class Autopilot {
  @SuppressWarnings("unused")
  private static final Logger m_logger = LoggerFactory.getLogger(Autopilot.class);
  private APProfile m_profile;

  private final double dt = 0.020;

  /**
   * Constructs an Autopilot from a given profile. This is the profile that the autopilot will use
   * for all actions.
   */
  public Autopilot(APProfile profile) {
    m_profile = profile;
  }

  /**
   * Returns the next field relative velocity for the trajectory
   *
   * @param current The robot's current position
   * @param velocity The robot's current (field relative) velocity
   * @param target The target the robot should drive towards
   */
  public Translation2d calculate(Pose2d current, Translation2d velocity, APTarget target) {
    // this method does not respect entry angle
    Translation2d offset = toTargetCoorinateFrame(
        target.m_reference.getTranslation().minus(current.getTranslation()), target);
    if (offset.equals(Translation2d.kZero)) {
      return Translation2d.kZero;
    }
    double dist = offset.getNorm();
    Translation2d towardsTarget = offset.div(dist);
    Translation2d initial = toTargetCoorinateFrame(velocity, target);
    Translation2d goal = towardsTarget.times(calculateMaxVelocity(dist, target.m_velocity));
    Translation2d out = correct(initial, goal);
    return toGlobalCoordinateFrame(out, target);
  }

  /**
   * Turns any other coordinate frame into a coordinate frame with positive x meaning in the
   * direction of the target's entry angle, if applicable (otherwise no change to
   * angles).
   */
  private Translation2d toTargetCoorinateFrame(Translation2d coords, APTarget target) {
    Rotation2d entryAngle = target.m_entryAngle.orElse(Rotation2d.kZero);
    return coords.rotateBy(entryAngle.unaryMinus());
  }

  /**
   * Turns a translation from a target-relative coordinate frame to a global coordinate frame
   */
  private Translation2d toGlobalCoordinateFrame(Translation2d coords, APTarget target) {
    Rotation2d entryAngle = target.m_entryAngle.orElse(Rotation2d.kZero);
    return coords.rotateBy(entryAngle);
  }

  /**
   * Determines the maximum velocity required to travel the given distance and end at rest.
   *
   * This uses constant acceleration, as determined by the value for I decceleration in the profile.
   */
  private double calculateMaxVelocity(double dist, double endVelo) {
    return Math.sqrt(Math.pow(endVelo, 2) + 2 * m_profile.m_constraintsI.m_decceleration * dist);
  }

  /**
   * Attempts to drive the initial translation to the goal translation using the parameters for
   * acceleration given in the profile
   */
  private Translation2d correct(Translation2d initial, Translation2d goal) {
    Rotation2d angleOffset = Rotation2d.kZero;
    if (!goal.equals(Translation2d.kZero)) {
      angleOffset = new Rotation2d(goal.getX(), goal.getY());
    }
    Translation2d adjustedGoal = goal.rotateBy(angleOffset.unaryMinus());
    Translation2d adjustedInitial = initial.rotateBy(angleOffset.unaryMinus());
    double initialI = adjustedInitial.getX();
    double initialU = adjustedInitial.getY();
    double goalI = adjustedGoal.getX();
    // we cap the adjusted I because we'd rather adjust now than overshoot.
    double adjustedI = Math.min(goalI,
        push(initialI, goalI, m_profile.m_constraintsI.m_acceleration));
    double adjustedU = push(initialU, 0, m_profile.m_constraintsU.m_acceleration);
    return new Translation2d(adjustedI, adjustedU).rotateBy(angleOffset);
  }

  private double push(double start, double end, double accel) {
    double maxChange = accel * dt;
    if (Math.abs(start - end) < maxChange) {
      return end;
    }
    if (start > end) {
      return start - maxChange;
    }
    return start + maxChange;
  }

  public boolean atSetpoint(Pose2d current, Pose2d goal) {
    boolean okXY = Math.hypot(current.getX() - goal.getX(),
        current.getY() - goal.getY()) <= m_profile.m_errorXY.in(Meters);
    boolean okTheta = Math.abs(current.getRotation().minus(goal.getRotation())
        .getRadians()) <= m_profile.m_errorTheta.in(Radians);
    return okXY && okTheta;
  }
}
