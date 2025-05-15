package frc.robot.subsystems.coral;

public interface CoralIO {
  void updateInputs(CoralIOInputs inputs);
  
  class CoralIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;
    public boolean frontCANrangeConnected = true;
    public boolean upperCANrangeConnected = true;
    public boolean innerCANrangeConnected = true;
    public double leftVoltage = 0.0;
    public double rightVoltage = 0.0;
    public double leftCurrent = 0.0;
    public double rightCurrent = 0.0;
    public double leftTemperature = 0.0;
    public double rightTemperature = 0.0;
    public boolean frontDetected = false;
    public boolean upperDetected = false;
    public boolean innerDetected = false;
  }

  void setLeftVoltage(double voltage);
  void setRightVoltage(double voltage);

  default void setVoltage(double voltage) {
    setRightVoltage(voltage);
    setLeftVoltage(voltage);
  }
}

