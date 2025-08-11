package frc.robot.vision.localization;

import java.util.Objects;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.vision.CameraIO;
import frc.robot.vision.CameraIOFactory;

public class CameraIOAprilTagSimFactory implements CameraIOFactory {
  private final Supplier<Pose2d> simPoseSupplier;

  public CameraIOAprilTagSimFactory(Supplier<Pose2d> simPoseSupplier) {
    this.simPoseSupplier = simPoseSupplier;
  }

  public CameraIO create(String name) {
    Transform3d robotToCamera = Objects.requireNonNull(
        LocalizationConstants.kCameras.get(name),
        "Camera " + name + " not found in camera map");
    return new CameraIOAprilTagSim(name, robotToCamera, simPoseSupplier);
  }
}
