package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;

public class Climber extends PassiveSubsystem implements AutoCloseable {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(Climber.class);
  private final TalonFX m_leftClimbMotor = new TalonFX(IDConstants.climbLeft);
  private final TalonFX m_rightClimbMotor = new TalonFX(IDConstants.climbRight);
  private final CANcoder m_encoder = new CANcoder(IDConstants.climbEncoder);

  private final Servo m_servo = new Servo(IDConstants.servo);

  private double m_voltage;
  private boolean m_voltageChanged;

  private final VoltageOut m_request = new VoltageOut(0);

  public Climber() {
    super();
    configMotors();
    configEncoder();
  }

  private void configEncoder() {
    m_encoder.getConfigurator().apply(ClimberConstants.encoderConfig);
  }

  private void configMotors() {
    m_leftClimbMotor.clearStickyFaults();
    m_rightClimbMotor.clearStickyFaults();
    m_leftClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
    m_rightClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
    m_rightClimbMotor
        .setControl(new Follower(IDConstants.climbLeft, ClimberConstants.rightMotorInvert));
  }

  /*
   * Opens the funnel, then resets the servo
   */
  public Command openFunnel() {
    return Commands.sequence(
        runOnce(() -> m_servo.set(ClimberConstants.k_openServoPosition)),
        Commands.waitSeconds(ClimberConstants.kFunnelOpenTime),
        runOnce(() -> m_servo.set(ClimberConstants.k_closedServoPosition)));
  }

  private void setMotor(double voltage) {
    m_voltageChanged = (m_voltage != voltage);
    m_voltage = voltage;
  }

  private void setUp() {
    setMotor(ClimberConstants.climberUpVolts);
  }

  private void setDown() {
    setMotor(ClimberConstants.climbDownVolts);
  }

  private void stop() {
    m_leftClimbMotor.stopMotor();
    m_voltage = 0.0;
    m_voltageChanged = false;
  }

  public double getPosition() {
    return m_leftClimbMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    if (m_voltageChanged) {
      m_leftClimbMotor.setControl(m_request.withOutput(m_voltage));
      m_voltageChanged = false;
    }
    SmartDashboard.putBoolean("Climb Ready", raised().getAsBoolean());
    SmartDashboard.putBoolean("Climbed", climbed().getAsBoolean());
    SmartDashboard.putNumber("climber pos", m_encoder.getPosition().getValueAsDouble());
  }

  public double getVelocity() {
    return m_leftClimbMotor.getVelocity().getValueAsDouble();
  }

  public Trigger climbed() {
    return new Trigger(() -> getPosition() <= ClimberConstants.climbPosition);
  }

  public Trigger raised() {
    return new Trigger(() -> getPosition() > ClimberConstants.climbReadyTolerance);
  }

  public Trigger lowered() {
    return new Trigger(() -> getPosition() <= ClimberConstants.stowPosition);
  }

  @Override
  public void close() throws Exception {
    m_leftClimbMotor.close();
    m_rightClimbMotor.close();
  }

  protected void passive() {}

  /**
   * Drives the cliber up until it has reached it's raised
   */
  public Command raise() {
    return Commands.sequence(
        runOnce(this::setUp),
        Commands.waitUntil(raised()))

        .finallyDo(this::stop);
  }

  /**
   * Drives the climber down until the climber is stowed
   */
  public Command lower() {
    return Commands.sequence(
        runOnce(this::setDown),
        Commands.waitUntil(lowered()))

        .finallyDo(this::stop);
  }

  /**
   * Drives the climber down until the climb position is reached 
   */
  public Command climb() {
    return Commands.sequence(
        runOnce(this::setDown),
        Commands.waitUntil(climbed()))

        .finallyDo(this::stop);
  }
}
