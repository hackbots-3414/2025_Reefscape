package frc.robot.subsystems.algae;

public interface AlgaeIO {
  void updateInputs(AlgaeIOInputs inputs);

  class AlgaeIOInputs {
    public boolean motorConnected = true;
    public double current = 0.0;
    public double torque = 0.0;
    public double voltage = 0.0;
    public double temperature = 0.0;
    public double stator = 0.0;
  }

  void setVoltage(double voltage);
}
