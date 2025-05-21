package frc.robot.subsystems.algae;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import frc.robot.subsystems.PassiveSubsystem;
import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;
import frc.robot.utils.LoopTimer;

public class Algae extends PassiveSubsystem {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(Algae.class);

  private final LoopTimer m_timer;

  private final AlgaeIO m_io;
  private AlgaeIOInputs m_inputs;

  private boolean m_hasAlgae;

  private MedianFilter m_filter = new MedianFilter(10);

  public Algae() {
    super();
    if (Robot.isReal()) {
      m_io = new AlgaeIOHardware();
    } else {
      m_io = new AlgaeIOSim();
    }
    m_inputs = new AlgaeIOInputs();
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
    m_hasAlgae = getTorqueCurrent() >= AlgaeConstants.kTorqueCurrentThreshold;
    // SmartDashboard.putBoolean("Algae/Held", m_hasAlgae);
    // SmartDashboard.putNumber("Algae/Torque", m_inputs.torque);
    // SmartDashboard.putNumber("Algae/Voltage", m_inputs.voltage);
    // SmartDashboard.putNumber("Algae/Temperature", m_inputs.temperature);
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
   * Ejects an algae with the correct conditions for a net score
   */
  public Command net() {
    return Commands.sequence(
        runOnce(() -> setVoltage(AlgaeConstants.kNetEjectVoltage)),
        Commands.waitSeconds(AlgaeConstants.kNetScoreTime))

        .finallyDo(this::keep)
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
        .onlyIf(holdingAlgae());
  }
}
