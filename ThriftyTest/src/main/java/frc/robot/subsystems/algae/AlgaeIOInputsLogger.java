package frc.robot.subsystems.algae;

import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;
import frc.robot.utils.OnboardLogger;

public class AlgaeIOInputsLogger {
  private final OnboardLogger log;

  public AlgaeIOInputsLogger(AlgaeIOInputs inputs) {
    log = new OnboardLogger("Algae");
    log.registerBoolean("Motor Connected", () -> inputs.motorConnected);
    log.registerDouble("Current", () -> inputs.current);
    log.registerDouble("Torque", () -> inputs.torque);
    log.registerDouble("Voltage", () -> inputs.voltage);
    log.registerDouble("Tempterature", () -> inputs.temperature);
    log.registerDouble("Stator Current", () -> inputs.stator);
  }

  public void log() {
    log.log();
  }
}
