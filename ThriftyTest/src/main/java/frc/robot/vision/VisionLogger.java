package frc.robot.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.LogBuilder.VisionLog;

public class VisionLogger {
    public static void record(VisionLog log) {
        SmartDashboard.putNumber(log.estimate().source(), log.error());
    }
}
