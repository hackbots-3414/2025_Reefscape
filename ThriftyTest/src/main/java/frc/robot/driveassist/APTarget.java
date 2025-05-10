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

  /**
   * Creates a blank autopilot target with reference (0,0) and rotation of zero.
   */
  public APTarget() {
    m_reference = Pose2d.kZero;
    m_entryAngle = Optional.empty();
    m_velocity = 0;
  }

  /**
   * Creates a new autopilot target with the given target pose, no entry angle, and no end velocity
   */
  public APTarget(Pose2d pose) {
    m_reference = pose;
    m_velocity = 0;
    m_entryAngle = Optional.empty();
  }

  /**
   * Modifies this instance's reference pose and returns itself for easier method chaining.
   * <i>NOTE:</i> This also sets, if unset, the entry angle to be the angle of the pose.
   */
  public APTarget withReference(Pose2d reference) {
    m_reference = reference;
    if (m_entryAngle == null) {
      m_entryAngle = Optional.of(reference.getRotation());
    }
    return this;
  }

  /**
   * Modifies this instance's entry angle and returns itself for easier method chaining
   */
  public APTarget withEntryAngle(Rotation2d entryAngle) {
    m_entryAngle = Optional.of(entryAngle);
    return this;
  }

  /**
   * Modifies this instance's end velocity and returns itself for easier method chaining
   */
  public APTarget withVelocity(double velocity) {
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
  public Optional<Rotation2d> getEntryAngle() {
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
  public APTarget flip() {
    m_reference = FlippingUtil.flipFieldPose(m_reference);
    m_entryAngle.ifPresent(rotation -> {
      m_entryAngle = Optional.of(FlippingUtil.flipFieldRotation(rotation));
    });
    return this;
  }
}
