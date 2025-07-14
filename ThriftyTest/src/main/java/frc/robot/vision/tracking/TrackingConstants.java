package frc.robot.vision.tracking;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

class TrackingConstants {
  public static final boolean kEnabled = true;

  protected static final String kCameraName = "algaeman";
  /* it is very important that the robot's position be ON THE GROUND (z = 0) */
  // protected static final Transform3d kRobotToCamera = new Transform3d(Units.inchesToMeters(9), Units.inchesToMeters(10), Units.inchesToMeters(11), Rotation3d.kZero);
  protected static final Transform3d kRobotToCamera = new Transform3d(Units.inchesToMeters(8.304 + 0.75), Units.inchesToMeters(9.75), Units.inchesToMeters(11), Rotation3d.kZero);

  /* simulation */
  protected static final double kFPS = 30;
  protected static final int kResWidth = 320;
  protected static final int kResHeight = 320;
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
