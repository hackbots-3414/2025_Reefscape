// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class Score extends Command {
  private final int level;
  private final Elevator elevator;
  private final Coral coral;
  
  public Score(int level, Elevator elevator, Coral coral) {
    this.level = level;
    this.elevator = elevator;
    this.coral = coral;
  }

  @Override
  public void initialize() {
    elevator.setLevel(level);
    SmartDashboard.putString("SCORE LOCATION", "L" + level);
  }

  @Override
  public void execute() {
    if (elevator.atSetpoint()) {
      coral.score();
    }
  }

  @Override
  public boolean isFinished() {
    return !coral.hasCoral();
  }
}
