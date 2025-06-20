package frc.robot.superstructure.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class SeedPose implements EnterableState {
  private final Pose2d m_pose;

  /**
   * Sets the robot's position to the center starting position
   */
  public SeedPose(Pose2d pose) {
    m_pose = pose;
  }

  public Command build(Subsystems subsystems) {
    return subsystems.drivetrain().seedLocal(m_pose);
  }

  public static SeedPose center() {
    return new SeedPose(FieldConstants.kStartCenter);
  }

  public static SeedPose left() {
    return new SeedPose(FieldConstants.kStartLeft);
  }

  public static SeedPose right() {
    return new SeedPose(FieldConstants.kStartRight);
  }
}
