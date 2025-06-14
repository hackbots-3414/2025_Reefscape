package frc.robot.utils;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.therekrab.autopilot.APTarget;

public class FieldUtils {
  /**
   * If the driver station reports that we are on the red alliance, flip the given pose. In other
   * words: Flips a pose either FROM alliance relative TO field relative or FROM field relative TO
   * alliance relative. Defaults to blue alliance.
   * <h1>IMPORTANT</h1> This depends on the current Driver Station settinng for alliance. When this
   * function is called, the driver station pose is read.
   */
  public static Pose2d getLocalPose(Pose2d localPose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      return FlippingUtil.flipFieldPose(localPose);
    } else {
      return localPose;
    }
  }

  /**
   * Flips the target if the driver station reports being on the red alliance
   */
  public static APTarget flipTargetConditionally(APTarget target) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      APTarget flipped = target.withReference(FlippingUtil.flipFieldPose(target.getReference()));
      // Use a weird thing
      if (target.getEntryAngle().isPresent()) {
        flipped = flipped.withEntryAngle(FlippingUtil.flipFieldRotation(target.getEntryAngle().get()));
      }
      return flipped;
    } else {
      return target;
    }
  }
}
