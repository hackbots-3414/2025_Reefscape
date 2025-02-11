package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.Constants.CommandBounds;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.RobotObserver;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class AlgaeScoreCommand extends Command {
    private AlgaeRollers rollers;
    private Elevator elevator;
    private Pivot pivot;
    private AlgaeLocationPresets location;
    private boolean isDone;
    private double initialTime;

    public AlgaeScoreCommand(AlgaeRollers rollers, Elevator elevator, Pivot pivot, AlgaeLocationPresets location) {
        this.rollers = rollers;
        this.elevator = elevator;
        this.pivot = pivot;
        this.location = location;
        addRequirements(rollers, elevator, pivot);
    }

    @Override
    public void initialize() {
        initialTime = Utils.getCurrentTimeSeconds();
        isDone = false;
        switch (location) {
            case NET:
                isDone = !RobotObserver.getShapeChecker().apply(CommandBounds.netBounds);
                elevator.setNet();
                pivot.setNet();
                break;
            case PROCESSOR:
                isDone = !RobotObserver.getShapeChecker().apply(CommandBounds.oppositeAllianceProcessorBounds);
                elevator.setProcessor();
                pivot.setProcessor();
                break;
            default:
                isDone = true;
        }
    }

    @Override
    public void execute() {
        if (elevator.atSetpoint() && pivot.atSetpoint()) {
            rollers.ejectAlgae();
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setStow();
        pivot.setStow();
        rollers.smartStop();
    }

    @Override
    public boolean isFinished() {
        return isDone || (Utils.getCurrentTimeSeconds() - initialTime) >= AlgaeRollerConstants.algaeEjectTime;
    }
}
