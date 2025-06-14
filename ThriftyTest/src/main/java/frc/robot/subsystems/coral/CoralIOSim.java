package frc.robot.subsystems.coral;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CoralIOSim implements CoralIO {
  private final DCMotorSim m_leftMotorSim;
  private final DCMotorSim m_rightMotorSim;

  private double m_leftVoltage;
  private double m_rightVoltage;

  public CoralIOSim() {
    m_leftMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1), 1, 0.1);
    m_rightMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1), 1, 0.1);
  }

  public void updateInputs(CoralIOInputs inputs) {
    m_leftMotorSim.update(Robot.kDefaultPeriod);
    m_rightMotorSim.update(Robot.kDefaultPeriod);
    inputs.leftCurrent = m_leftMotorSim.getCurrentDrawAmps();
    inputs.rightCurrent = m_rightMotorSim.getCurrentDrawAmps();
    inputs.leftTemperature = 0.0;
    inputs.rightTemperature = 0.0;
    inputs.leftVoltage = m_leftVoltage;
    inputs.rightVoltage = m_rightVoltage;
    inputs.frontDetected = SmartDashboard.getBoolean("Coral/Front CANrange", false);
    inputs.upperDetected = SmartDashboard.getBoolean("Coral/Upper CANrange", false);
    inputs.innerDetected = SmartDashboard.getBoolean("Coral/Inner CANrange", false);
    // Without publishing these values, they will never be able to be read.
    SmartDashboard.putBoolean("Coral/Front CANrange", inputs.frontDetected);
    SmartDashboard.putBoolean("Coral/Upper CANrange", inputs.upperDetected);
    SmartDashboard.putBoolean("Coral/Inner CANrange", inputs.innerDetected);
  }

  public void setLeftVoltage(double voltage) {
    m_leftVoltage = voltage;
    m_leftMotorSim.setInputVoltage(voltage);
  }

  public void setRightVoltage(double voltage) {
    m_rightVoltage = voltage;
    m_rightMotorSim.setInputVoltage(voltage);
  }
}
