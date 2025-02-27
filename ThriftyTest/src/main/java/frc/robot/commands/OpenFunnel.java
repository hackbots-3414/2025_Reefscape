// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OpenFunnel extends Command {
  private Climber m_climber;
  private int timeRemaining;
  /** Creates a new OpenFunnel. */
  public OpenFunnel(Climber climber) {
    m_climber = climber;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.openFunnel();
    timeRemaining = 3;
  }

  @Override
  public void execute() {
    timeRemaining --;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeRemaining == 0;
  }
}
