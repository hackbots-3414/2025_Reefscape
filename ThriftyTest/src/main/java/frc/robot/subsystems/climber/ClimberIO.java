package frc.robot.subsystems.climber;

public interface ClimberIO {
  void updateInputs(ClimberIOInputs inputs);

  public class ClimberIOInputs {
    public boolean leftConnected = true;
    public boolean rightConnected = true;
    public double leftCurrent = 0.0;
    public double rightCurrent = 0.0;
    public double leftVoltage = 0.0;
    public double rightVoltage = 0.0;
    public double leftTemp = 0.0;
    public double rightTemp = 0.0;
    public double leftVelocityRPS = 0.0;
    public double rightVelocityRPS = 0.0;
    public boolean encoderConnected = true;
    public double position = 0.0;
  }

  void setVoltage(double voltage);
  void setServo(double position);
}
