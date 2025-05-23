package frc.robot.subsystems.pivot;

public interface PivotIO {
  void updateInputs(PivotIOInputs inputs);

  class PivotIOInputs {
    public boolean motorConnected = true;
    public double current = 0.0;
    public double voltage = 0.0;
    public double temperatue = 0.0;
    public double position = 0.0;
  }

  void setPosition(double position, boolean holdingAlgae);
}
