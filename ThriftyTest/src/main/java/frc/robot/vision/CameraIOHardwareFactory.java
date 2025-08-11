package frc.robot.vision;

public class CameraIOHardwareFactory implements CameraIOFactory {
  public CameraIO create(String name) {
    return new CameraIOHardware(name);
  }
}
