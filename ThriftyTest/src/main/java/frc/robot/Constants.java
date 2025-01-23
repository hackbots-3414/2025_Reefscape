package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static class DriveConstants {
        public static final PIDConstants k_translationPID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants k_rotationPID = new PIDConstants(5.0, 0.0, 0.0);
    }
    public static class VisionConstants {
        // The camera names
        public static Map<String, Transform3d> cameras = Map.ofEntries(
            Map.entry("Camera", new Transform3d(
                new Translation3d(-0.52, -0.52, 1.0),
                new Rotation3d(0,0,Math.PI/6)
            ))
            // Map.entry("Front Camera", new Transform3d(
            //     new Translation3d(0.52, 0.52, 1.0),
            //     new Rotation3d(0, 0, Math.PI)
            // ))
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
    }
    public static class FieldConstants {
        public static final Distance k_fieldWidth = Meters.of(8.05);
        public static final Distance k_fieldLength = Meters.of(17.55);
    }
}
