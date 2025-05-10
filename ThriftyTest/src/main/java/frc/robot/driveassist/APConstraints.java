package frc.robot.driveassist;

/**
 * A class that holds constrain information for an autopilot action
 * Constraints are max acceleration and decelleration.
 */
public class APConstraints {
  protected double m_acceleration;
  protected double m_decceleration;

  public APConstraints() {}

  public APConstraints(double acceleration, double decceleration) {
    m_acceleration = acceleration;
    m_decceleration = decceleration;
  }

  public APConstraints withAcceleration(double acceleration) {
    m_acceleration = acceleration;
    return this;
  }

  public APConstraints withDecceleration(double decceleration) {
    m_decceleration = decceleration;
    return this;
  }
}
