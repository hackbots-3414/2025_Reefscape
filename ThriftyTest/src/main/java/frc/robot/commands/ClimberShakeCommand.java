// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberShakeCommand extends Command {
  private final Climber m_climber;

  private int m_ticks;

  /** Creates a new ClimberShakeCommand. */
  public ClimberShakeCommand(Climber climber) {
    addRequirements(climber);
    m_climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if we are below the shake point we go back up. If we're at climb ready, we go down.
    if (m_climber.getPosition() < ClimberConstants.kShakePosition) {
      m_climber.setUp();
    } else if (m_climber.atClimb()) {
      m_climber.setDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
