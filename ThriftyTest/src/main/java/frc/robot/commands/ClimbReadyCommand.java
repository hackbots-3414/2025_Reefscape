// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbReadyCommand extends Command {
  private final Climber m_climber;

  /** Creates a new ClimbReadyCommand. */
  public ClimbReadyCommand(Climber climber) {
    addRequirements(climber);
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.climbReady();
  }
}
