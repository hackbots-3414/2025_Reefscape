package frc.robot.vision.tracking;

import static edu.wpi.first.units.Units.Milliseconds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Time;

class TrackingConstants {
  protected static final String kCameraName = "camera";
  protected static final Transform3d kRobotToCamera = Transform3d.kZero;

  /* simulation */
  protected static final double kFPS = 24;
  protected static final int kResWidth = 640;
  protected static final int kResHeight = 480;
  protected static final Rotation2d kFOVDiag = Rotation2d.fromDegrees(100);

  protected static final double kCalibError = 0.5;
  protected static final double kCalibStdDevs = 0.3;
  protected static final Time kLatency = Milliseconds.of(17);
  protected static final Time kLatencyStdDevs = Milliseconds.of(6);
}
