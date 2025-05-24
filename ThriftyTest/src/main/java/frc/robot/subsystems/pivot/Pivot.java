// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import frc.robot.subsystems.PassiveSubsystem;
import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;
import frc.robot.utils.LoopTimer;
import frc.robot.utils.OnboardLogger;

public class Pivot extends PassiveSubsystem {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(Pivot.class);
  private final OnboardLogger m_ologger;

  private final LoopTimer m_timer;

  private final PivotIO m_io;
  private final PivotIOInputs m_inputs;
  private final PivotIOInputsLogger m_inputsLogger;

  private PivotState m_reference;

  public Pivot() {
    super();
    if (Robot.isReal()) {
      m_io = new PivotIOHardware();
    } else {
      m_io = new PivotIOSim();
    }
    m_inputs = new PivotIOInputs();
    m_inputsLogger = new PivotIOInputsLogger(m_inputs);
    m_reference = PivotState.Stow;
    m_ologger = new OnboardLogger("Pivot");
    m_ologger.registerString("State", m_reference::toString);
    m_ologger.registerBoolean("Ready", ready());
    m_timer = new LoopTimer("Pivot");
  }


  private void setPosition(PivotState state) {
    take();
    m_io.setPosition(state.position(), RobotObserver.getAlgaePieceHeld());
    m_reference = state;
  }

  public Trigger ready() {
    return new Trigger(() -> Math.abs(m_reference.position() - m_inputs.position) < PivotConstants.kTolerance);
  }

  @Override
  public void periodic() {
    m_timer.reset();
    m_io.updateInputs(m_inputs);
    m_inputsLogger.log();
    m_ologger.log();
    m_timer.log();
  }

  protected void passive() {
    setPosition(PivotState.Stow);
  }

  /**
   * Drives the pivot to the given PivotState This command ends when the state is reached.
   */
  public Command go(PivotState state) {
    return Commands.sequence(
        runOnce(() -> setPosition(state)),
        Commands.waitUntil(ready()));
  }
}
