package frc.robot.subsystems.climber;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.PassiveSubsystem;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.utils.LoopTimer;
import frc.robot.utils.OnboardLogger;

public class Climber extends PassiveSubsystem {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(Climber.class);
  private final OnboardLogger m_ologger;

  private final LoopTimer m_timer;

  private final ClimberIO m_io;
  private final ClimberIOInputs m_inputs;
  private final ClimberIOInputsLogger m_inputsLogger;

  public Climber() {
    super();
    if (Robot.isReal()) {
      m_io = new ClimberIOHardware();
    } else {
      m_io = new ClimberIOSim();
    }
    m_inputs = new ClimberIOInputs();
    m_inputsLogger = new ClimberIOInputsLogger(m_inputs);
    m_ologger = new OnboardLogger("Climber");
    m_ologger.registerBoolean("Climbed", climbed());
    m_ologger.registerBoolean("Raised", raised());
    m_ologger.registerBoolean("Lowered", lowered());
    m_timer = new LoopTimer("Climber");
  }

  /*
   * Opens the funnel, then resets the servo
   */
  public Command openFunnel() {
    return Commands.sequence(
        runOnce(() -> m_io.setServo(ClimberConstants.kOpenServoPosition)),
        Commands.waitSeconds(ClimberConstants.kFunnelOpenTime),
        runOnce(() -> m_io.setServo(ClimberConstants.kClosedServoPosition)));
  }

  private void setMotor(double voltage) {
    take();
    m_io.setVoltage(voltage);
  }

  private void setUp() {
    setMotor(ClimberConstants.kUpVolts);
  }

  private void setDown() {
    setMotor(ClimberConstants.kDownVolts);
  }

  private void stop() {
    setMotor(0);
  }

  @Override
  public void periodic() {
    m_timer.reset();
    m_io.updateInputs(m_inputs);
    m_inputsLogger.log();
    m_ologger.log();
    m_timer.log();
  }

  public Trigger climbed() {
    return new Trigger(() -> m_inputs.position <= ClimberConstants.kClimbPosition);
  }

  public Trigger raised() {
    return new Trigger(() -> m_inputs.position > ClimberConstants.kClimbReadyTolerance);
  }

  public Trigger lowered() {
    return new Trigger(() -> m_inputs.position <= ClimberConstants.kStowPosition);
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
