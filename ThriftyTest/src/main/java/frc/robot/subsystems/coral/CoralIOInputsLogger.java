package frc.robot.subsystems.coral;

import frc.robot.subsystems.coral.CoralIO.CoralIOInputs;
import frc.robot.utils.OnboardLogger;

public class CoralIOInputsLogger {
  private final OnboardLogger log;

  public CoralIOInputsLogger(CoralIOInputs inputs) {
    log = new OnboardLogger("Coral");
    log.registerBoolean("Left Motor Connected", () -> inputs.leftMotorConnected);
    log.registerBoolean("Right Motor Connected", () -> inputs.rightMotorConnected);
    log.registerBoolean("Front CANrange Connected", () -> inputs.frontCANrangeConnected);
    log.registerBoolean("Upper CANrange Connected", () -> inputs.upperCANrangeConnected);
    log.registerBoolean("Inner CANrange Connected", () -> inputs.innerCANrangeConnected);
    log.registerDouble("Left Voltage", () -> inputs.leftVoltage);
    log.registerDouble("Right Voltage", () -> inputs.rightVoltage);
    log.registerDouble("Left Current", () -> inputs.leftCurrent);
    log.registerDouble("Right Current", () -> inputs.rightCurrent);
    log.registerDouble("Left Temperature", () -> inputs.leftTemperature);
    log.registerDouble("Right Temperature", () -> inputs.rightTemperature);
    log.registerBoolean("Front Detected", () -> inputs.frontDetected);
    log.registerBoolean("Upper Detected", () -> inputs.upperDetected);
    log.registerBoolean("Inner Detected", () -> inputs.innerDetected);
    log.registerDouble("Front Distance", () -> inputs.frontDistance);
    log.registerDouble("Upper Distance", () -> inputs.upperDistance);
    log.registerDouble("Inner Distance", () -> inputs.innerDistance);
    log.registerDouble("Front Strength", () -> inputs.frontStrength);
    log.registerDouble("Upper Strength", () -> inputs.upperStreingth);
    log.registerDouble("Inner Strength", () -> inputs.innerStrength);
  }

  public void log() {
    log.log();
  }
}
