package frc.robot.vision;

import frc.robot.utils.OnboardLogger;
import frc.robot.vision.CameraIO.CameraIOInputs;

public class CameraIOInputsLogger {
  private final OnboardLogger log;

  public CameraIOInputsLogger(CameraIOInputs inputs, String name) {
    log = new OnboardLogger("Cameras");
    log.registerBoolean(name + "Connected", () -> inputs.connected);
  }

  public void log() {
    log.log();
  }
}
