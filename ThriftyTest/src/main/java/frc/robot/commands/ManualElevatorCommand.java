package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ManualElevatorCommand extends Command {
    private final boolean isUp;
    private final Elevator elevator;

    public ManualElevatorCommand(Elevator elevator, boolean isUp) {
        this.isUp = isUp;
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setSpeed(isUp ? ElevatorConstants.manualUpSpeed : ElevatorConstants.manualDownSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
