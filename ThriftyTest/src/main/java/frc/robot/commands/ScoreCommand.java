// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ScoreCommand extends Command {
    private final int elevatorLevel;
    private final Elevator elevator;
    // private final Coral coral;

    public ScoreCommand(int elevatorLevel, Elevator elevator) {
        this.elevatorLevel = elevatorLevel;
        this.elevator = elevator;
        // this.coral = coral;
    }

    @Override
    public void initialize() {
        elevator.setLevel(elevatorLevel);
    }

    @Override
    public void execute() {
        // if (elevator.atSetpoint()) {
        // coral.score();
        // }
    }

    @Override
    public boolean isFinished() {
        // return !coral.hasCoral();
        // return true;
        return elevator.atSetpoint();
    }
}
