package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends Command {
    private final boolean isUp;
    private final Elevator elevator;

    public ManualElevator(Elevator elevator, boolean isUp) {
        this.isUp = isUp;
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.disableStateSpace();
    }

    @Override
    public void execute() {
        elevator.setSpeed(isUp ? 0.1 : -0.1);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.enableStateSpace();
    }
}
