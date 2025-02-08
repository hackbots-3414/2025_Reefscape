package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class RobotState {
    /* Robot Position */
    public static Pose2d m_pose = new Pose2d();

    /* A field2d is useful for managing the pose of the robot */
    public static Field2d m_field = new Field2d();
}
