package frc.robot.subsystems.climber;

import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.utils.OnboardLogger;

public class ClimberIOInputsLogger {
  private final OnboardLogger log;

  public ClimberIOInputsLogger(ClimberIOInputs inputs) {
    log = new OnboardLogger("Climber");
    log.registerBoolean("Left Motor Connected", () -> inputs.leftConnected);
    log.registerBoolean("Right Motor Connected", () -> inputs.rightConnected);
    log.registerDouble("Left Current", () -> inputs.leftCurrent);
    log.registerDouble("Right Current", () -> inputs.rightCurrent);
    log.registerDouble("Left Voltage", () -> inputs.leftVoltage);
    log.registerDouble("Right Voltage", () -> inputs.rightVoltage);
    log.registerDouble("Left Temperature", () -> inputs.leftTemp);
    log.registerDouble("Right Temperature", () -> inputs.rightTemp);
    log.registerDouble("Left Velocity RPS", () -> inputs.leftVelocityRPS);
    log.registerDouble("Right Velocity RPS", () -> inputs.rightVelocityRPS);
    log.registerBoolean("CANcoder Connected", () -> inputs.encoderConnected);
    log.registerDouble("Position", () -> inputs.position);
  }

  public void log() {
    log.log();
  }
}
