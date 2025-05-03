package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Robot;
import frc.robot.RobotObserver;

public class AlgaeRollers extends PassiveSubsystem implements AutoCloseable {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(AlgaeRollers.class);

  private final TalonFX m_algaeRoller = new TalonFX(IDConstants.algae);

  private double m_voltage;
  private boolean m_voltageChanged;

  private boolean m_hasAlgae;

  private MedianFilter m_filter = new MedianFilter(10);

  public AlgaeRollers() {
    super();
    configIntakeMotor();
    RobotObserver.setAlgaePieceHeldSupplier(this.holdingAlgae());
  }

  private void configIntakeMotor() {
    m_algaeRoller.clearStickyFaults();
    m_algaeRoller.getConfigurator().apply(AlgaeRollerConstants.motorConfig);
  }

  private void setMotor(double voltage) {
    if (voltage != m_voltage) {
      m_voltageChanged = true;
    }
    m_voltage = voltage;
  }

  public Trigger holdingAlgae() {
    return new Trigger(() -> m_hasAlgae);
  }

  private double getTorqueCurrent() {
    double measurement = m_algaeRoller.getTorqueCurrent().getValueAsDouble();
    return m_filter.calculate(measurement);
  }

  private void stop() {
    setMotor(0);
  }

  /**
   * If <code>shouldHold</code> is true, then try to hold an algae. Otherwise, stop the motors.
   */
  private void keep(boolean shouldHold) {
    if (shouldHold) {
      setMotor(AlgaeRollerConstants.holdVoltage);
    } else {
      stop();
    }
  }

  private void updateObjectState() {
    if (Robot.isReal()) {
      m_hasAlgae = getTorqueCurrent() >= AlgaeRollerConstants.torqueCurrentThreshold;
    } else {
      m_hasAlgae = SmartDashboard.getBoolean("Algae Held", false);
    }

    SmartDashboard.putBoolean("Algae Held", m_hasAlgae);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Temp", m_algaeRoller.getDeviceTemp().getValueAsDouble());
    updateObjectState();
    if (m_voltageChanged) {
      m_algaeRoller.setVoltage(m_voltage);
      m_voltageChanged = false;
    }
  }

  @Override
  public void close() throws Exception {
    m_algaeRoller.close();
  }

  protected void passive() {}

  /**
   * Intakes an algae, then holds it. If an algae is already held, the command does not run.
   */
  public Command intake() {
    return Commands.sequence(
        runOnce(() -> setMotor(AlgaeRollerConstants.intakeVoltage)),
        Commands.waitUntil(holdingAlgae()))

        .finallyDo(this::keep)
        .unless(holdingAlgae());
  }

  /**
   * Ejects an algae with the correct conditions for a net score
   */
  public Command net() {
    return Commands.sequence(
        runOnce(() -> setMotor(AlgaeRollerConstants.netEjectVoltage)),
        Commands.waitSeconds(AlgaeRollerConstants.algaeEjectTime))

        .finallyDo(this::keep)
        .onlyIf(holdingAlgae());
  }

  /**
   * Ejects an algae with the correct conditions for a processor score
   */
  public Command processorScore() {
    return Commands.sequence(
        runOnce(() -> setMotor(AlgaeRollerConstants.processorEjectVoltage)),
        Commands.waitSeconds(AlgaeRollerConstants.processorScoreTime))

        .finallyDo(this::keep)
        .onlyIf(holdingAlgae());
  }
}
