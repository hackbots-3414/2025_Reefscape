package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

public class Constants {
    public static class DriveConstants {
        public static final PIDConstants k_translationPID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants k_rotationPID = new PIDConstants(5.0, 0.0, 0.0);
    }
}
