package frc.robot.driveassist;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * A class representing a profile that completely determines how AP approaches a target.
 *
 * A profile includes two APConstraints, one for the direction towards the target, and one for the
 * direction perpendicular to the target.
 *
 * A profile also includes a maximum error, in the XY plane as well as rotation.
 */
public class APProfile {
  protected APConstraints m_pathConstraints;
  protected APConstraints m_correctionConstraints;
  protected Distance m_errorXY;
  protected Angle m_errorTheta;

  public APProfile() {
    m_errorXY = Meters.of(0);
    m_errorTheta = Rotations.of(0);
  }

  /**
   * Modifies this profile's tolerated error in the XY plane and returns itself
   */
  public APProfile withErrorXY(Distance errorXY) {
    m_errorXY = errorXY;
    return this;
  }

  /**
   * Modifies this profile's tolerated angular error and returns itself
   */
  public APProfile withErrorTheta(Angle errorTheta) {
    m_errorTheta = errorTheta;
    return this;
  }

  /**
   * Modifies this profile's path generation constraints and returns itself
   */
  public APProfile withPathConstraints(APConstraints constraintsI) {
    m_pathConstraints = constraintsI;
    return this;
  }

  /**
   * Modifies this profile's correction constraints and returns itself
   */
  public APProfile withCorrectionConstraints(APConstraints constraintsU) {
    m_correctionConstraints = constraintsU;
    return this;
  }

  /**
   * Returns the tolerated translation error for this profile
   */
  public Distance getErrorXY() {
    return m_errorXY;
  }

  /**
   * Returns the tolerated angular error for this profile
   */
  public Angle getErrorTheta() {
    return m_errorTheta;
  }

  /**
   * Returns the path generation constraints for this profile
   */
  public APConstraints getPathConstraints() {
    return m_pathConstraints;
  }

  /**
   * Returns the correction constraints for this profile
   */
  public APConstraints getCorrectionConstraints() {
    return m_correctionConstraints;
  }
}
