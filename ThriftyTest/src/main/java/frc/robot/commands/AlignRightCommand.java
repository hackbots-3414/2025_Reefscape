// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CanRangeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRightCommand extends Command {

  public boolean alignedleft;
  public double distance = -1.0;
  public boolean isDone = false;
  private CommandSwerveDrivetrain m_drivetrain;
  private ChassisSpeeds alignDriveSpeed = new ChassisSpeeds(0, -0.5, 0);

  /** Creates a new AlignCommand. */
  public AlignRightCommand(CommandSwerveDrivetrain m_drivetrain) {
    this.m_drivetrain = m_drivetrain;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = m_drivetrain.getRangeRightDistance(); //  Need to change 
     }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = m_drivetrain.getRangeRightDistance();
      m_drivetrain.driveRobotRelative(alignDriveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance > CanRangeConstants.farAlignedDistanceMeters;
  }
}
