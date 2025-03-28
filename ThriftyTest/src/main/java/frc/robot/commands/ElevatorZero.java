// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotObserver;
import frc.robot.subsystems.Elevator;

public class ElevatorZero extends Command {
  private boolean applied;

  private final Elevator m_elevator;
  /** Creates a new ElevatorZero. */
  public ElevatorZero(Elevator elevator) {
    m_elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    applied = false;
    m_elevator.setStow();
  }

  @Override
  public void execute() {
    if (m_elevator.atSetpoint() && !RobotObserver.getCoralPieceHeld()) {
      applied = true;
      m_elevator.prepZero();
    } 
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted && !RobotObserver.getCoralPieceHeld()) m_elevator.zeroElevator();
    m_elevator.enableLimits();
    m_elevator.release();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atZero() && applied || RobotObserver.getCoralPieceHeld();
  }
}
