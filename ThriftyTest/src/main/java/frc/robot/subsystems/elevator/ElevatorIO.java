package frc.robot.subsystems.elevator;

public interface ElevatorIO {
  void updateInputs(ElevatorIOInputs inputs);

  public class ElevatorIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;
    public double leftVoltage = 0.0;
    public double rightVoltage = 0.0;
    public double leftCurrent = 0.0;
    public double rightCurrent = 0.0;
    public double leftTemp = 0.0;
    public double rightTemp = 0.0;
    public double leftVelocityRPS = 0.0;
    public double rightVelocityRPS = 0.0;
    public double leftPosition = 0.0;
    public double rightPosition = 0.0;
    public double position = 0.0;
    public double reference = 0.0;
    public boolean zeroCANrangeConnected = true;
    public boolean zeroCANrangeDetected = false;
    public double zeroCANrangeDistance = 0.0;
    public double zeroCANrangeStrength = 0.0;
  }

  void setPosition(double position);

  void setVoltage(double voltage);

  void disableLimits();
  void enableLimits();

  void calibrateZero();
}
