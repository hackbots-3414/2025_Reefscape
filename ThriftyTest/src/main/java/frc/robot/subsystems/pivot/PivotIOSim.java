package frc.robot.subsystems.pivot;

public class PivotIOSim implements PivotIO {
  private double m_position;

  public void updateInputs(PivotIOInputs inputs) {
    inputs.position = m_position;
  }

  public void setPosition(double position, boolean holdingAlgae) {
    m_position = position;
  }
}
