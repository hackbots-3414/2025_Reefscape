package frc.robot.subsystems.elevator;

import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.utils.OnboardLogger;

public class ElevatorIOInputsLogger {
  private final OnboardLogger log;

  public ElevatorIOInputsLogger(ElevatorIOInputs inputs) {
    log = new OnboardLogger("Elevator");
    log.registerBoolean("Left Motor Connected", () -> inputs.leftMotorConnected);
    log.registerBoolean("Right Motor Connected", () -> inputs.rightMotorConnected);
    log.registerDouble("Left Voltage", () -> inputs.leftVoltage);
    log.registerDouble("Right Voltage", () -> inputs.rightVoltage);
    log.registerDouble("Left Current", () -> inputs.leftCurrent);
    log.registerDouble("Right Current", () -> inputs.rightCurrent);
    log.registerDouble("Left Temperature", () -> inputs.leftTemp);
    log.registerDouble("Right Tempterature", () -> inputs.rightTemp);
    log.registerDouble("Left Velocity RPS", () -> inputs.leftVelocityRPS);
    log.registerDouble("Right Velocity RPS", () -> inputs.rightVelocityRPS);
    log.registerDouble("Left Position", () -> inputs.leftPosition);
    log.registerDouble("Right Position", () -> inputs.rightPosition);
    log.registerDouble("Position", () -> inputs.position);
    log.registerDouble("Reference", () -> inputs.reference);
    log.registerBoolean("Zero CANrange Connected", () -> inputs.zeroCANrangeConnected);
    log.registerBoolean("Zero CANrange Detected", () -> inputs.zeroCANrangeDetected);
    log.registerDouble("Zero CANrange Distance", () -> inputs.zeroCANrangeDistance);
    log.registerDouble("Zero CANrange Strength", () -> inputs.zeroCANrangeStrength);
  }

  public void log() {
    log.log();
  }
}
