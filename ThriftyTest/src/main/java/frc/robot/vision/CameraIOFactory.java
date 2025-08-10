package frc.robot.vision;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.vision.localization.CameraIOAprilTagSim;
import frc.robot.vision.tracking.CameraIOTrackingSim;

public class CameraIOFactory {
  private final Supplier<Pose2d> simPoseSupplier;

  public CameraIOFactory(Supplier<Pose2d> simPoseSupplier) {
    this.simPoseSupplier = simPoseSupplier;
  }

  public CameraIO hardware(String name) {
    return new CameraIOHardware(name);
  }

  public CameraIO aprilTagSim(String name, Transform3d robotToCamera) {
    return new CameraIOAprilTagSim(name, robotToCamera, simPoseSupplier);
  }

  public CameraIO trackingSim(String name, Transform3d robotToCamera) {
    return new CameraIOTrackingSim(name, robotToCamera, simPoseSupplier);
  }
}
