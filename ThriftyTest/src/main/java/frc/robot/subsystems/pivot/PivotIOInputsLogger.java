package frc.robot.subsystems.pivot;

import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;
import frc.robot.utils.OnboardLogger;

public class PivotIOInputsLogger {
  private final OnboardLogger log;

  public PivotIOInputsLogger(PivotIOInputs inputs) {
    log = new OnboardLogger("Pivot");
    log.registerBoolean("Motor Connected", () -> inputs.motorConnected);
    log.registerDouble("Current", () -> inputs.current);
    log.registerDouble("Voltage", () -> inputs.voltage);
    log.registerDouble("Temperature", () -> inputs.temperatue);
    log.registerDouble("Position", () -> inputs.position);
  }

  public void log() {
    log.log();
  }
}
