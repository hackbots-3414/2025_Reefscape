package frc.robot.subsystems.pivot;

public class PivotIOSim implements PivotIO {
  private double m_position = 0;
  private double m_reference = 0;

  public void updateInputs(PivotIOInputs inputs) {
    m_position = (m_position + m_reference) / 2.0;
    inputs.position = m_position;
  }

  public void setPosition(double position, boolean holdingAlgae) {
    m_reference = position;
  }
}
