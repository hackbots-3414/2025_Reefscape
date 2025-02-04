// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorToPointCommand extends Command {
    private final double elevatorPosition;
    private final Elevator elevator;
    public ElevatorToPointCommand(double elevatorPosition, Elevator elevator) {
        this.elevatorPosition = elevatorPosition;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(elevatorPosition);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
