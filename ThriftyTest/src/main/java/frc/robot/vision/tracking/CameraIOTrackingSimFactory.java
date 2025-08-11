package frc.robot.vision.tracking;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.CameraIO;
import frc.robot.vision.CameraIOFactory;

public class CameraIOTrackingSimFactory implements CameraIOFactory {
  private final Supplier<Pose2d> simPoseSupplier;

  public CameraIOTrackingSimFactory(Supplier<Pose2d> simPoseSupplier) {
    this.simPoseSupplier = simPoseSupplier;
  }

  public CameraIO create(String name) {
    return new CameraIOTrackingSim(name, TrackingConstants.kRobotToCamera, simPoseSupplier);
  }
}
