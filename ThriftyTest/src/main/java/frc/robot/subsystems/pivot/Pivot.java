// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import frc.robot.subsystems.PassiveSubsystem;
import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;

public class Pivot extends PassiveSubsystem {
  private final PivotIO m_io;
  private PivotIOInputs m_inputs;

  private PivotState m_reference;

  public Pivot() {
    super();
    if (Robot.isReal()) {
      m_io = new PivotIOHardware();
    } else {
      m_io = new PivotIOSim();
    }
    m_inputs = new PivotIOInputs();
    m_reference = PivotState.Stow;
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
    m_io.updateInputs(m_inputs);
    SmartDashboard.putBoolean("Pivot/Ready", ready().getAsBoolean());
    SmartDashboard.putString("Pivot/Reference", m_reference.toString());
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
