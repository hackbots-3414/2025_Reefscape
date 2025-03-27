// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorZero extends Command {
  private final Elevator m_elevator;
  /** Creates a new ElevatorZero. */
  public ElevatorZero(Elevator elevator) {
    m_elevator = elevator;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.goDownNoStopping();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.zeroElevator();
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atZero();
  }
}
