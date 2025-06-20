package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
  private double m_position = 0;
  private double m_reference = 0;

  public void updateInputs(ElevatorIOInputs inputs) {
    m_position = (m_position + m_reference) / 2.0;
    inputs.position = m_position;
    inputs.reference = m_reference;
    inputs.zeroCANrangeDetected = m_position < 1e-3;
  }

  public void setPosition(double position) {
    m_reference = position; // that's an awesome elevator! it's instant!
  }

  public void setVoltage(double voltage) {}
  
  public void enableLimits() {}
  public void disableLimits() {}

  public void calibrateZero() {}

}
