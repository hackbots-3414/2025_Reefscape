package frc.robot.driveassist;

import java.util.Optional;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A class representing the goal end state of an autopilot action
 * 
 * A target needs a reference Pose2d, but can optionally have a specified entry angle
 *
 * The target also may have a desired end velocity.
 */
public class APTarget {
  protected Pose2d m_reference;
  protected Optional<Rotation2d> m_entryAngle;
  protected double m_velocity;
  protected Optional<Double> m_rotationRadius;

  /**
   * Creates a blank autopilot target with reference (0,0) and rotation of zero.
   */
  public APTarget() {
    m_reference = Pose2d.kZero;
    m_entryAngle = Optional.empty();
    m_velocity = 0;
    m_rotationRadius = Optional.empty();
  }

  /**
   * Creates a new autopilot target with the given target pose, no entry angle, and no end velocity
   */
  public APTarget(Pose2d pose) {
    m_reference = pose;
    m_velocity = 0;
    m_entryAngle = Optional.empty();
    m_rotationRadius = Optional.empty();
  }

  /**
   * Modifies this instance's reference pose and returns itself for easier method chaining.
   */
  public APTarget withReference(Pose2d reference) {
    APTarget target = this.clone();
    target.m_reference = reference;
    return target;
  }

  /**
   * Modifies this instance's entry angle and returns itself for easier method chaining
   */
  public APTarget withEntryAngle(Rotation2d entryAngle) {
    APTarget target = this.clone();
    target.m_entryAngle = Optional.of(entryAngle);
    return target;
  }

  /**
   * Modifies this instance's end velocity and returns itself for easier method chaining
   */
  public APTarget withVelocity(double velocity) {
    APTarget target = this.clone();
    target.m_velocity = velocity;
    return target;
  }

  /**
   * Modifies this instance's rotation radius and returns itself for easier method chaining
   *
   * Rotation radius is the distance from the target pose that rotation goals are respected. By
   * default, rotation goals are always respected, but if autopilot shouldn't reorient the robot
   * until X distance from setpoint, this can be used to make that change.
   */
  public APTarget withRotationRadius(double radius) {
    APTarget copy = this.clone();
    copy.m_rotationRadius = Optional.of(radius);
    return copy;
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
  public Optional<Rotation2d> getEntryAngle() {
    return m_entryAngle;
  }

  /**
   * Returns this target's end velocity
   */
  public double getVelocity() {
    return m_velocity;
  }

  /**
   * Returns this target's rotation radius
   */
  public Optional<Double> getRotationRadius() {
    return m_rotationRadius;
  }

  /**
   * Flips a target across the field, preserving relative entry angle and rotation.
   */
  public APTarget flip() {
    Pose2d ref = FlippingUtil.flipFieldPose(m_reference);
    APTarget target = new APTarget(ref);
    m_entryAngle.ifPresent(rotation -> {
      Rotation2d entry = FlippingUtil.flipFieldRotation(rotation);
      target.m_entryAngle = Optional.of(entry);
    });
    target.m_rotationRadius = m_rotationRadius;
    return target;
  }

  /**
   * Creates a copy of this APTarget
   */
  public APTarget clone() {
    APTarget target = new APTarget();
    target.m_reference = m_reference;
    target.m_velocity = m_velocity;
    target.m_entryAngle = m_entryAngle;
    target.m_rotationRadius = m_rotationRadius;
    return target;
  }
}
