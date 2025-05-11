package frc.robot.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import java.util.Map;
import java.util.Set;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class VisionConstants {
  public static final boolean enableVision = true;
  public static final boolean k_enableLogging = true;

  public static final double k_rotationCoefficient = Math.PI * 20;
  public static final double k_translationCoefficient = 0.10; // previously 0.10

  public static final AprilTagFieldLayout k_layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final String k_estimationName = "estimation";
  public static final String kRejectedName = "rejected";

  public static final String k_logPath = "/home/lvuser/logs/vision";
  public static final String k_simLogPath = "logs/vision";

  private static final double k_moduleHeight = 0.190;

  private static final double k_tightPitch = -Units.degreesToRadians(22.5);
  private static final double k_widePitch = -Units.degreesToRadians(25.0);

  private static final double k_tightYaw = Units.degreesToRadians(37.0); // this doesn't seem
                                                                         // right
  private static final double k_wideYaw = Units.degreesToRadians(-7.0);

  // The camera names
  public static Map<String, Transform3d> fakecameras = Map.ofEntries(
      Map.entry("test", new Transform3d(0, 0, 0, new Rotation3d())));
  public static Map<String, Transform3d> cameras = Map.ofEntries(
      Map.entry("cam1", new Transform3d( // left tight
          new Translation3d(0.256, 0.289, k_moduleHeight),
          new Rotation3d(0, k_tightPitch, -k_tightYaw))),
      Map.entry("cam2", new Transform3d( // left wide
          new Translation3d(0.337, 0.331, k_moduleHeight),
          new Rotation3d(0, k_widePitch, -k_wideYaw))),
      Map.entry("cam3", new Transform3d( // right wide
          new Translation3d(0.337, -0.331, k_moduleHeight),
          new Rotation3d(0, k_widePitch, k_wideYaw))),
      Map.entry("cam4", new Transform3d( // right tight
          new Translation3d(0.256, -0.289, k_moduleHeight),
          new Rotation3d(0, k_tightPitch, k_tightYaw))));

  public static final String k_leftAlignName = "cam1";
  public static final String k_rightAlignName = "cam4";

  // The tick time for each pose estimator to run
  public static final double k_periodic = 0.02;
  // The maximum number of results (per camera)
  public static final double k_expectedResults = 10;
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
  public static final double k_distanceMultiplier = 5.0;
  public static final double k_noisyDistance = 4.0;
  public static final double k_ambiguityMultiplier = 0.4;
  public static final double k_ambiguityShifter = 0.2;
  public static final double k_targetMultiplier = 80;
  public static final double k_differenceThreshold = 0.10;
  public static final double k_differenceMultiplier = 200.0;
  public static final double k_latencyMultiplier = 1.3;

  public static final double k_headingThreshold = Units.degreesToRadians(3);

  // Stats about the camera for simulation
  public static final int k_resWidth = 320 * 2;
  public static final int k_resHeight = 240 * 2;
  public static final Rotation2d k_fov = Rotation2d.fromDegrees(82.0);
  public static final Rotation2d kHorizontalFov = Rotation2d.fromDegrees(70.0);


  // Simulated error:
  public static final Time k_avgLatency = Milliseconds.of(18);
  public static final Time k_latencyStdDev = Milliseconds.of(2);
  public static final double k_avgErr = 0.08;
  public static final double k_errStdDev = 0.02;

  // Stop using vision after X time
  public static final double k_visionTimeout = 0.5;

  // reef tag ids (single tag only)
  public static final Set<Integer> k_reefIds = Set.of(
      6, 7, 8, 9, 10, 11, // red tags
      17, 18, 19, 20, 21, 22 // blue tags
  );
}

