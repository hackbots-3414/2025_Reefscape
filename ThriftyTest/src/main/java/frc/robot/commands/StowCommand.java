// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StowCommand extends Command {
  private final Pivot m_pivot;
  private final Elevator m_elevator;

  /** Creates a new StowCommand. */
  public StowCommand(Elevator elevator, Pivot pivot) {
    m_pivot = pivot;
    m_elevator = elevator;
    addRequirements(m_pivot, m_elevator);
  }

  @Override
  public void initialize() {
    m_pivot.setStow();
    m_elevator.setStow();
  }

  @Override
  public void end(boolean interrupted) {
    m_pivot.stop();
  }

  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint() && m_pivot.atSetpoint();
  }
}
