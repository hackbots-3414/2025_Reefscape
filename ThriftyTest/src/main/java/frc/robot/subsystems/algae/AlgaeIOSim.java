package frc.robot.subsystems.algae;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class AlgaeIOSim implements AlgaeIO {
  private final DCMotorSim m_motorSim;
  
  private double m_voltage;

  public AlgaeIOSim() {
    m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
        DCMotor.getKrakenX60(1), 0.1, 0.1);
  }

  public void updateInputs(AlgaeIOInputs inputs) {
    m_motorSim.update(Robot.kDefaultPeriod);
    inputs.voltage = m_voltage;
    inputs.current = m_motorSim.getCurrentDrawAmps();
    inputs.torque = SmartDashboard.getNumber("Algae/Torque", 0.0);
  }

  public void setVoltage(double voltage) {
    m_voltage = voltage;
    m_motorSim.setInputVoltage(voltage);
  }
}
