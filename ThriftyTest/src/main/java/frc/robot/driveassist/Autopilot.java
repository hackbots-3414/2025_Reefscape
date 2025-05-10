package frc.robot.driveassist;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Autopilot {
  @SuppressWarnings("unused")
  private static final Logger m_logger = LoggerFactory.getLogger(Autopilot.class);
  private Profile m_profile;

  private final double dt = 0.020;

  public Autopilot(Profile profile) {
    m_profile = profile;
  }

  /**
   * Returns the next field relative velocity for the trajectory
   *
   * @param current The robot's current position
   * @param velocity The robot's current (field relative) velocity
   * @param target The target the robot should drive towards
   */
  public Translation2d calculate(
      Pose2d current,
      Translation2d velocity,
      Target target) {
    Pose2d reference = target.m_reference;
    Rotation2d entryAngle = target.m_entryAngle;
    // direction and distance to actual target
    Translation2d offset = reference.getTranslation().minus(current.getTranslation());
    double distance = offset.getNorm();
    if (distance == 0)
      return Translation2d.kZero;
    // create new target
    Translation2d entryDirection = new Translation2d(
        entryAngle.getCos(),
        entryAngle.getSin());
    // end velocity
    Translation2d endVelocity = entryDirection.times(target.m_velocity);
    // calculate directions for i & j
    Translation2d directionI = offset.div(distance);
    Translation2d directionU = new Translation2d(
        directionI.getY(),
        -directionI.getX());
    // current velocity (i & j)
    double veloI = project(velocity, directionI);
    double veloU = 0.0;
    double entryDistance = 0;
    if (target.getEntryAngle() != null) {
      veloU = project(velocity, directionU);
      Translation2d entry = entryDirection.times(-distance);
      entryDistance = project(entry, directionU);
    }


    // drive towards goal state
    double adjustedI = approach(distance, veloI, m_profile.getConstraintsI())
        + Math.max(project(endVelocity, directionI), 0);
    double adjustedU = approach(entryDistance, veloU, m_profile.getConstraintsU());
    // combine
    Translation2d adjusted = directionI.times(adjustedI).plus(
        directionU.times(adjustedU));
    return adjusted;
  }

  private double project(Translation2d vector, Translation2d axis) {
    double dot = vector.getX() * axis.getX() + vector.getY() * axis.getY();
    return dot / Math.pow(axis.getNorm(), 2);
  }
 
  private double approach(double distance, double initial, Constraints c) {
    double goal = Math.sqrt(2 * c.m_decceleration * Math.abs(distance)) * Math.signum(distance);
    if (Math.abs(goal - initial) < dt * c.m_acceleration) {
      // we're within range, just adjust to what we need.
      return goal;
   }
    // check for a "out-of-bounds" position
    if (goal < initial && goal > 0)
      return goal;
    if (goal > initial && goal < 0)
      return goal;
    if (goal > initial) {
      return initial + dt * c.m_acceleration;
    } else {
      return initial - dt * c.m_acceleration;
    }
  }

  public boolean atSetpoint(Pose2d current, Pose2d goal) {
    boolean okXY = Math.hypot(current.getX() - goal.getX(),
        current.getY() - goal.getY()) <= m_profile.m_errorXY.in(Meters);
    boolean okTheta = Math.abs(current.getRotation().minus(goal.getRotation())
        .getRadians()) <= m_profile.m_errorTheta.in(Radians);
    return okXY && okTheta;
  }

  /* Constraints to limit autopilot */
  public static class Constraints {
    private double m_acceleration;
    private double m_decceleration;

    public Constraints() {}

    public Constraints(double acceleration, double decceleration) {
      m_acceleration = acceleration;
      m_decceleration = decceleration;
    }

    public Constraints withAcceleration(double acceleration) {
      m_acceleration = acceleration;
      return this;
    }

    public Constraints withDecceleration(double decceleration) {
      m_decceleration = decceleration;
      return this;
    }
  }

  /* Profile (how to reach the goal) */
  public static class Profile {
    private Constraints m_constraintsI;
    private Constraints m_constraintsU;
    private Distance m_errorXY;
    private Angle m_errorTheta;

    public Profile() {
      m_errorXY = Meters.of(0);
      m_errorTheta = Rotations.of(0);
    }

    public Profile withErrorXY(Distance errorXY) {
      m_errorXY = errorXY;
      return this;
    }

    public Profile withErrorTheta(Angle errorTheta) {
      m_errorTheta = errorTheta;
      return this;
    }

    public Profile withConstraintsI(Constraints constraintsI) {
      m_constraintsI = constraintsI;
      return this;
    }

    public Profile withConstraintsU(Constraints constraintsU) {
      m_constraintsU = constraintsU;
      return this;
    }

    public Distance getErrorXY() {
      return m_errorXY;
    }

    public Angle getErrorTheta() {
      return m_errorTheta;
    }

    public Constraints getConstraintsI() {
      return m_constraintsI;
    }

    public Constraints getConstraintsU() {
      return m_constraintsU;
    }
  }

  /* End States (the goal to reach) */
  public static class Target {
    private Pose2d m_reference;
    private Rotation2d m_entryAngle;
    private double m_velocity;

    /**
     * Creates a blank autopilot target with reference (0,0) and rotation of zero.
     */
    public Target() {
      m_reference = Pose2d.kZero;
      m_entryAngle = null;
      m_velocity = 0;
    }

    /**
     * Creates a new autopilot target with the given target pose, no entry angle, and no end
     * velocity
     */
    public Target(Pose2d pose) {
      m_reference = pose;
      m_velocity = 0;
      m_entryAngle = null;
    }

    /**
     * Modifies this instance's reference pose and returns itself for easier method chaining.
     * <i>NOTE:</i> This also sets, if unset, the entry angle to be the angle of the pose.
     */
    public Target withReference(Pose2d reference) {
      m_reference = reference;
      if (m_entryAngle == null)
        m_entryAngle = reference.getRotation();
      return this;
    }

    /**
     * Modifies this instance's entry angle and returns itself for easier method chaining
     */
    public Target withEntryAngle(Rotation2d entryAngle) {
      m_entryAngle = entryAngle;
      return this;
    }

    /**
     * Modifies this instance's end velocity and returns itself for easier method chaining
     */
    public Target withVelocity(double velocity) {
      m_velocity = velocity;
      return this;
    }

    /**
     * Returns this target's reference pose
     */
    public Pose2d getReference() {
      return m_reference;
    }

    /**
     * Returns this target's desired entry angle
     */
    public Rotation2d getEntryAngle() {
      return m_entryAngle;
    }

    /**
     * Returns this target/s end velocity
     */
    public double getVelocity() {
      return m_velocity;
    }

    /**
     * Flips a target across the field, preserving relative entry angle and rotation.
     */
    public Target flip() {
      m_reference = FlippingUtil.flipFieldPose(m_reference);
      m_entryAngle = FlippingUtil.flipFieldRotation(m_entryAngle);
      return this;
    }
  }
}
