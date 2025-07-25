// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralLevel;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import frc.robot.subsystems.PassiveSubsystem;
import frc.robot.subsystems.coral.CoralIO.CoralIOInputs;
import frc.robot.utils.LoopTimer;

public class Coral extends PassiveSubsystem {
  private final Logger m_logger = LoggerFactory.getLogger(Coral.class);

  private final LoopTimer m_timer;

  private final CoralIO m_io;
  private final CoralIOInputs m_inputs;
  private final CoralIOInputsLogger m_inputsLogger;

  public Coral() {
    super();
    if (Robot.isReal()) {
      m_io = new CoralIOHardware();
    } else {
      m_io = new CoralIOSim();
    }
    m_inputs = new CoralIOInputs();
    m_inputsLogger = new CoralIOInputsLogger(m_inputs);
    RobotObserver.setCoralHeldSupplier(held());
    m_timer = new LoopTimer("Coral");
  }

  private void setVoltage(double voltage) {
    take();
    m_io.setVoltage(voltage);
  }

  private void setIntake() {
    setVoltage(CoralConstants.kIntakeVoltage);
  }

  private void setL2Score() {
    m_logger.trace("Setting L2 eject");
    setVoltage(CoralConstants.kL2EjectVoltage);
  }

  private void setL3Score() {
    m_logger.trace("Setting L3 eject");
    setVoltage(CoralConstants.kL3EjectVoltage);
  }

  private void setL4Score() {
    m_logger.trace("Setting L4 eject");
    setVoltage(CoralConstants.kL4EjectVoltage);
  }

  private void setL1Score() {
    m_io.setLeftVoltage(CoralConstants.kL1LeftEjectVoltage);
    m_io.setRightVoltage(CoralConstants.kL1RightEjectVoltage);
  }

  private void stop() {
    setVoltage(0);
  }

  public Trigger present() {
    return new Trigger(
        () -> m_inputs.upperDetected || m_inputs.innerDetected || m_inputs.frontDetected);
  }

  public Trigger held() {
    return new Trigger(() -> m_inputs.frontDetected && !m_inputs.upperDetected);
  }

  @Override
  public void periodic() {
    m_timer.reset();
    m_io.updateInputs(m_inputs);
    m_inputsLogger.log();
    m_timer.log();
  }

  protected void passive() {
    if (present().getAsBoolean() && !held().getAsBoolean()) {
      setIntake();
    } else {
      stop();
    }
  }

  /**
   * Intakes a game piece. The command ends when the piece is fully in the robot.
   */
  public Command intake() {
    return Commands.sequence(
        runOnce(this::setIntake),
        Commands.waitUntil(held()))

        .finallyDo(this::stop)
        .unless(held());
  }

  public Command score(CoralLevel level) {
    return Commands.sequence(
        runOnce(() -> {
          switch (level) {
            case L1, SecondaryL1 -> setL1Score();
            case L2 -> setL2Score();
            case L3 -> setL3Score();
            case L4 -> setL4Score();
          }
        }),
        Commands.waitUntil(held().negate()))

        .onlyIf(held());
  }

  /**
   * Ejects a coral piece
   */
  public Command eject() {
    return Commands.sequence(
        runOnce(() -> setVoltage(CoralConstants.kEjectVoltage)),
        Commands.waitUntil(present().negate()))

        .finallyDo(this::stop)
        .onlyIf(present());
  }

}
