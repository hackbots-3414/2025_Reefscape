package frc.robot.utils;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtils {
    public static Pose2d flipPose(Pose2d pose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return FlippingUtil.flipFieldPose(pose);
        } else {
            return pose;
        }
    }
}
