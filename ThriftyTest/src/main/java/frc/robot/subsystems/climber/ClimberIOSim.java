package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim m_lefMotorSim;
  private final DCMotorSim m_rightMotorSim;

  private double m_voltage;

  public ClimberIOSim() {
    m_lefMotorSim = new DCMotorSim(ClimberConstants.kPlant, DCMotor.getKrakenX60(1), 0.1, 0.1);
    m_rightMotorSim = new DCMotorSim(ClimberConstants.kPlant, DCMotor.getKrakenX60(1), 0.1, 0.1);
    m_lefMotorSim.setAngle(Units.rotationsToRadians(ClimberConstants.kStowPosition));
  }

  public void updateInputs(ClimberIOInputs inputs) {
    m_lefMotorSim.update(Robot.kDefaultPeriod);
    m_rightMotorSim.update(Robot.kDefaultPeriod);
    inputs.leftVoltage = m_voltage;
    inputs.rightVoltage = m_voltage;
    inputs.leftCurrent = m_lefMotorSim.getCurrentDrawAmps();
    inputs.rightCurrent = m_rightMotorSim.getCurrentDrawAmps();
    inputs.leftVelocityRPS = m_lefMotorSim.getAngularVelocityRPM() / 60.0;
    inputs.rightVelocityRPS = m_rightMotorSim.getAngularVelocityRPM() / 60.0;
    inputs.position = m_lefMotorSim.getAngularPositionRotations();
  }

  public void setVoltage(double voltage) {
    m_voltage = voltage;
    m_lefMotorSim.setInputVoltage(voltage);
    m_rightMotorSim.setInputVoltage(voltage);
  }

  public void setServo(double position) {}
}
