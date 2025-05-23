package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
  private double m_position = Double.NaN;

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.position = m_position;
    inputs.reference = m_position;
    inputs.zeroCANrangeDetected = m_position == 0;
  }

  public void setPosition(double position) {
    m_position = position; // that's an awesome elevator! it's instant!
  }

  public void setVoltage(double voltage) {}
  
  public void enableLimits() {}
  public void disableLimits() {}

  public void calibrateZero() {}

}
