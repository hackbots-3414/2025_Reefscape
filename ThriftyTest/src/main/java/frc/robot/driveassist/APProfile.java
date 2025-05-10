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
  protected APConstraints m_constraintsI;
  protected APConstraints m_constraintsU;
  protected Distance m_errorXY;
  protected Angle m_errorTheta;

  public APProfile() {
    m_errorXY = Meters.of(0);
    m_errorTheta = Rotations.of(0);
  }

  public APProfile withErrorXY(Distance errorXY) {
    m_errorXY = errorXY;
    return this;
  }

  public APProfile withErrorTheta(Angle errorTheta) {
    m_errorTheta = errorTheta;
    return this;
  }

  public APProfile withConstraintsI(APConstraints constraintsI) {
    m_constraintsI = constraintsI;
    return this;
  }

  public APProfile withConstraintsU(APConstraints constraintsU) {
    m_constraintsU = constraintsU;
    return this;
  }

  public Distance getErrorXY() {
    return m_errorXY;
  }

  public Angle getErrorTheta() {
    return m_errorTheta;
  }

  public APConstraints getConstraintsI() {
    return m_constraintsI;
  }

  public APConstraints getConstraintsU() {
    return m_constraintsU;
  }
}
