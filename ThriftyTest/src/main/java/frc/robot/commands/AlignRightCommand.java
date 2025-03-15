// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.core.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.CanRangeConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRightCommand extends Command {

  public boolean alignedleft;
  private StatusSignal<Distance> range;
  public double distance = range.getValueAsDouble();
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
    distance = range.getValueAsDouble(); //  Need to change 
     }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = range.getValueAsDouble();
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
