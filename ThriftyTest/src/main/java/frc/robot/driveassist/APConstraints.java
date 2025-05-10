package frc.robot.driveassist;

/**
 * A class that holds constrain information for an autopilot action Constraints are max acceleration
 * and decelleration.
 */
public class APConstraints {
  protected double m_acceleration;
  protected double m_decceleration;

  public APConstraints() {}

  public APConstraints(double acceleration, double decceleration) {
    m_acceleration = acceleration;
    m_decceleration = decceleration;
  }

  public static APConstraints unlimited() {
    return new APConstraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
  }

  /**
   * Modifies this constraint's max acceleration value. This affects the maximum acceleration
   * that the autopilot action will use to correct initial velocities.
   */
  public APConstraints withAcceleration(double acceleration) {
    m_acceleration = acceleration;
    return this;
  }

  /**
   * Modifies this constraint's decceleration value.
   * 
   * This is only necessary if the constraint is used for I control
   */
  public APConstraints withDecceleration(double decceleration) {
    m_decceleration = decceleration;
    return this;
  }
}
