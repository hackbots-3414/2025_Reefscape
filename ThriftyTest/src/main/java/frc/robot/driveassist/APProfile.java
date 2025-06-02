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
  protected APConstraints pathConstraints;
  protected APConstraints correctionConstraints;
  protected Distance errorXY;
  protected Angle errorTheta;
  protected Distance beelineRadius;

  public APProfile() {
    errorXY = Meters.of(0);
    errorTheta = Rotations.of(0);
    beelineRadius = Meters.of(0);
  }

  /**
   * Modifies this profile's tolerated error in the XY plane and returns itself
   */
  public APProfile withErrorXY(Distance errorXY) {
    this.errorXY = errorXY;
    return this;
  }

  /**
   * Modifies this profile's tolerated angular error and returns itself
   */
  public APProfile withErrorTheta(Angle errorTheta) {
    this.errorTheta = errorTheta;
    return this;
  }

  /**
   * Modifies this profile's path generation constraints and returns itself
   */
  public APProfile withPathConstraints(APConstraints pathConstraints) {
    this.pathConstraints = pathConstraints;
    return this;
  }

  /**
   * Modifies this profile's correction constraints and returns itself
   */
  public APProfile withCorrectionConstraints(APConstraints correctionConstraints) {
    this.correctionConstraints = correctionConstraints;
    return this;
  }

  /**
   * Modifies this profile's beeline radius and returns itself
   *
   * The beeline radius is a distance where, under that range, entry angle is no longer respected.
   * This prevents small overshoots from causing the robot to make a full arc and instaed correct
   * itself.
   */
  public APProfile withBeelineRadius(Distance beelineRadius) {
    this.beelineRadius = beelineRadius;
    return this;
  }

  /**
   * Returns the tolerated translation error for this profile
   */
  public Distance getErrorXY() {
    return errorXY;
  }

  /**
   * Returns the tolerated angular error for this profile
   */
  public Angle getErrorTheta() {
    return errorTheta;
  }

  /**
   * Returns the path generation constraints for this profile
   */
  public APConstraints getPathConstraints() {
    return pathConstraints;
  }

  /**
   * Returns the correction constraints for this profile
   */
  public APConstraints getCorrectionConstraints() {
    return correctionConstraints;
  }

  /**
   * Returns the beeline radius for this profile
   */
  public Distance getBeelineRadius() {
    return beelineRadius;
  }
}
