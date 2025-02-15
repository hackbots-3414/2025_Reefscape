package frc.robot.utils;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtils {
    public static Pose2d flipPose(Pose2d pose) {
        try {
            if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
                return FlippingUtil.flipFieldPose(pose);
            } else {
                return pose;
            }
        } catch (Exception e) {
            return pose;
        }
    }
}
