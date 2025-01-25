package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import java.util.Map;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/*
PLEASE READ:

To ensure consistency throughout the code, the same coordinate system is used
here as is specified in WPILib's documentation.

To read it all, check this out:
https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html

TL;DR:
We use NWU axes convention.
This means that, when viewed from above, the North, West, and then upwards will
correspond to +X, +Y, and +Z respectively.

Example:
           +X
            ^
            |
        |-front-|
        |       |
+Y <--- |       |
        |       |
        |-------|

And +Z is upwards, so it wouldn't show here.
*/

public class Constants {
    public static class RobotConstants {
        public static final double k_robotX = Units.inchesToMeters(20.0);
        public static final double k_robotY = Units.inchesToMeters(20.0);

        public static final double k_cameraOffsetX = k_robotX / 2.0;
        public static final double k_cameraOffsetY = k_robotY / 2.0;
        
        public static final double k_cameraHeight = Units.inchesToMeters(9.0);
        public static final double k_cameraPitch = -Math.PI / 12.0;
    }
    public static class DriveConstants {
        public static final PIDConstants k_translationPID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants k_rotationPID = new PIDConstants(5.0, 0.0, 0.0);
    }
    public static class VisionConstants {
        // aliases
        private static final double x = RobotConstants.k_cameraOffsetX;
        private static final double y = RobotConstants.k_cameraOffsetY;
        private static final double z = RobotConstants.k_cameraHeight;
        private static final double pitch = RobotConstants.k_cameraPitch;

        // The camera names
        public static Map<String, Transform3d> cameras = Map.ofEntries(
            Map.entry("0", new Transform3d(
                new Translation3d(x, y, z),
                new Rotation3d(0, pitch, Math.PI/6 /* 30 degrees */)
            )),
            Map.entry("1", new Transform3d(
                new Translation3d(x, y, z),
                new Rotation3d(0, pitch, Math.PI / 3 /* 60 degrees */)
            )),
            Map.entry("2", new Transform3d(
                new Translation3d(-x, y, z),
                new Rotation3d(0, pitch, 2.0 * Math.PI / 3.0)
            )),
            Map.entry("3", new Transform3d(
                new Translation3d(-x, y, z),
                new Rotation3d(0, pitch, 5.0 * Math.PI / 6.0)
            )),
            Map.entry("4", new Transform3d(
                new Translation3d(-x, -y, z),
                new Rotation3d(0, pitch, -5.0 * Math.PI / 6.0)
            )),
            Map.entry("5", new Transform3d(
                new Translation3d(-x, -y, z),
                new Rotation3d(0, pitch, -2.0 * Math.PI / 3.0)
            )),
            Map.entry("6", new Transform3d(
                new Translation3d(x, -y, z),
                new Rotation3d(0, pitch, -Math.PI / 3.0)
            )),
            Map.entry("7", new Transform3d(
                new Translation3d(x, -y, z),
                new Rotation3d(0, pitch, -Math.PI / 6.0)
            ))
        );
        // The tick time for each pose estimator to run
        public static final double k_periodic = 0.02;
        // The maximum number of results (per camera) we expect to see per tick
        public static final double k_maxResults = 3;
        // The maximum tolerated latency, in seconds.
        public static final double k_latencyThreshold = 0.75;
        // The maximum tolerated ambiguity value.
        public static final double k_AmbiguityThreshold = 0.2;
        // The farthest out off a field a pose estimate can say we are
        // (in each dimension separately)
        public static final Distance k_XYMargin = Meters.of(0.5);
        // The maximum distance from 0 that a camera's pose can report
        public static final Distance k_ZMargin = Meters.of(1.5);
        // Some configuration variables:
        public static final boolean k_useStdDevs = true;
        public static final double k_distanceMultiplier = 7.0;
        public static final double k_noisyDistance = 4.0;
        public static final double k_ambiguityMultiplier = 0.4;
        public static final double k_ambiguityShifter = 0.2;
        public static final double k_targetMultiplier = 10;
        // Stats about the camera for simulation
        public static final int k_resWidth = 320;
        public static final int k_resHeight = 240;
        public static final Rotation2d k_fov = Rotation2d.fromDegrees(70.0);
        // Simulated error:
        public static final Time k_avgLatency = Milliseconds.of(18);
        public static final Time k_latencyStdDev = Milliseconds.of(5);
        public static final double k_avgErr = 0.03;
        public static final double k_errStdDev = 0.02;
    }
    public static class FieldConstants {
        public static final Distance k_fieldWidth = Meters.of(8.05);
        public static final Distance k_fieldLength = Meters.of(17.55);
    }
}
