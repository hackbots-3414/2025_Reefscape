package frc.robot.driveassist;

/**
 * A class that holds constrain information for an autopilot action Constraints are max acceleration
 * and decelleration.
 */
public class APConstraints {
  protected double acceleration;
  protected double jerk;

  /** Creates a blank APConstraints */
  public APConstraints() {}

  /**
   * Creates a new APCosntraints with given acceleration and deceleration */
  public APConstraints(double acceleration, double deceleration) {
    acceleration = acceleration;
    jerk = deceleration;
  }

  /** Unlimited constraints */
  public static APConstraints unlimited() {
    return new APConstraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
  }

  /**
   * Modifies this constraint's max acceleration value and returns itself. This affects the maximum
   * acceleration that the autopilot action will use to correct initial velocities.
   */
  public APConstraints withAcceleration(double acceleration) {
    acceleration = acceleration;
    return this;
  }

  /**
   * Modifies this constraint's max jerk value and returns itself. Higher values mean a faster
   * deceleration.
   *
   * This is only used at the end of an autopilot action, not the beginning.
   */
  public APConstraints withJerk(double jerk) {
    jerk = jerk;
    return this;
  }
}
