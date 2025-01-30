// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorToPointCommand extends Command {
    private final double elevatorPosition;
    private final Elevator elevator;
    private  double timer = 0;

    public ElevatorToPointCommand(double elevatorPosition, Elevator elevator) {
        this.elevatorPosition = elevatorPosition;
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.setPosition(elevatorPosition);
        timer = 0;
    }

    @Override
    public void execute() {
        timer += 0.02;
    }

    @Override
    public boolean isFinished() {
        return timer > 1;
    }
}
