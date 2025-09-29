package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Seconds;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import frc.robot.utils.SuperDebouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import frc.robot.subsystems.PassiveSubsystem;
import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;
import frc.robot.utils.LoopTimer;
import frc.robot.utils.OnboardLogger;
import frc.robot.utils.SuperDebouncer;

public class Algae extends PassiveSubsystem {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(Algae.class);

  private final OnboardLogger m_ologger;

  private final LoopTimer m_timer;

  private final AlgaeIO m_io;
  private AlgaeIOInputs m_inputs;
  private AlgaeIOInputsLogger m_inputsLogger;

  private boolean m_hasAlgae;
  private SuperDebouncer m_debouncer =
      new SuperDebouncer(AlgaeConstants.kAlgaeDebounceTime.in(Seconds), DebounceType.kFalling);

  private MedianFilter m_filter = new MedianFilter(10);

  public Algae() {
    super();
    if (Robot.isReal()) {
      m_io = new AlgaeIOHardware();
    } else {
      m_io = new AlgaeIOSim();
    }
    resetDebouncer(false);
    m_inputs = new AlgaeIOInputs();
    m_inputsLogger = new AlgaeIOInputsLogger(m_inputs);
    m_ologger = new OnboardLogger("Algae");
    m_ologger.registerBoolean("Holding", holdingAlgae());

    RobotObserver.setAlgaePieceHeldSupplier(this.holdingAlgae());
    m_timer = new LoopTimer("Algae");
  }

  private void setVoltage(double voltage) {
    take();
    m_io.setVoltage(voltage);
  }

  public Trigger holdingAlgae() {
    return new Trigger(() -> m_hasAlgae);
  }

  private double getTorqueCurrent() {
    return m_filter.calculate(m_inputs.torque);
  }

  private void stop() {
    setVoltage(0);
  }

  /**
   * If <code>shouldHold</code> is true, then try to hold an algae. Otherwise, stop the motors.
   */
  private void keep(boolean shouldHold) {
    if (shouldHold) {
      setVoltage(AlgaeConstants.kHoldVoltage);
    } else {
      stop();
    }
  }


  @Override
  public void periodic() {
    m_timer.reset();
    m_io.updateInputs(m_inputs);
    m_inputsLogger.log();
    m_hasAlgae =
        m_debouncer.calculate(getTorqueCurrent() >= AlgaeConstants.kTorqueCurrentThreshold);
    m_ologger.log();
    m_timer.log();
  }

  protected void passive() {
    keep(holdingAlgae().getAsBoolean());
  }

  /**
   * Intakes an algae, then holds it. If an algae is already held, the command does not run.
   */
  public Command intake() {
    return Commands.sequence(
        runOnce(() -> setVoltage(AlgaeConstants.kIntakeVoltage)),
        Commands.waitUntil(holdingAlgae()))

        .finallyDo(() -> keep(holdingAlgae().getAsBoolean()))
        .unless(holdingAlgae());
  }

  /**
   * Resets the debouncer to be willing to accept any new inputs as accurate
   */
  private void resetDebouncer(boolean ignore) {
    if (ignore) {
      return;
    }
    m_debouncer.overrideTimer();
  }

  /**
   * Ejects an algae with the correct conditions for a net score
   */
  public Command net() {
    return Commands.sequence(
        runOnce(() -> setVoltage(AlgaeConstants.kNetEjectVoltage)),
        Commands.waitSeconds(AlgaeConstants.kNetScoreTime))

        .finallyDo(this::keep)
        .finallyDo(this::resetDebouncer)
        .onlyIf(holdingAlgae());
  }

  /**
   * Ejects an algae with the correct conditions for a processor score
   */
  public Command processorScore() {
    return Commands.sequence(
        runOnce(() -> setVoltage(AlgaeConstants.kProcessorEjectVoltage)),
        Commands.waitSeconds(AlgaeConstants.kProcessorScoreTime))

        .finallyDo(this::keep)
        .finallyDo(this::resetDebouncer)
        .onlyIf(holdingAlgae());
  }

  /** Ejects an algae fast */
  public Command eject() {
    return Commands.sequence(
        runOnce(() -> setVoltage(AlgaeConstants.kManualEjectVoltage)),
        Commands.waitSeconds(AlgaeConstants.kManualEjectTime))

        .finallyDo(this::keep)
        .finallyDo(this::resetDebouncer)
        .onlyIf(holdingAlgae());
  }

}
