package frc.robot.vision.tracking;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

class TrackingConstants {
  public static final boolean kEnabled = true;

  protected static final String kCameraName = "camera";
  /* it is very important that the robot's position be ON THE GROUND (z = 0) */
  protected static final Transform3d kRobotToCamera = new Transform3d(0, 0, 0.3, Rotation3d.kZero);

  /* simulation */
  protected static final double kFPS = 14;
  protected static final int kResWidth = 640;
  protected static final int kResHeight = 480;
  protected static final Rotation2d kFOVDiag = Rotation2d.fromDegrees(100);

  protected static final double kCalibError = 0.5;
  protected static final double kCalibStdDevs = 0.3;
  protected static final Time kLatency = Milliseconds.of(17);
  protected static final Time kLatencyStdDevs = Milliseconds.of(6);

  /*
   * algae location constants - these measure the distance from the ground to the center of the
   * algae
   */
  protected static final Distance kGroundAlgaeHeight = Inches.of(8.125);
  protected static final Distance kLollipopAlgaeHeight = Inches.of(14.0625);

  protected static final boolean kDistanceEstimationEnabled = true;

  protected static final Time kExpirationTime = Milliseconds.of(300);
}
