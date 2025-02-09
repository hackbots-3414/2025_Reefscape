package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotObserver;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.Constants.CommandBounds;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class AlgaeIntakeCommand extends Command {
    private AlgaeRollers rollers;
    private Elevator elevator;
    private Pivot pivot;
    private CommandSwerveDrivetrain drivetrain;
    private AlgaeLocationPresets location;
    private boolean isDone;

    public AlgaeIntakeCommand(AlgaeRollers rollers, Elevator elevator, Pivot pivot, CommandSwerveDrivetrain drivetrain,
            AlgaeLocationPresets location) {
        this.rollers = rollers;
        this.elevator = elevator;
        this.pivot = pivot;
        this.drivetrain = drivetrain;
        this.location = location;
        addRequirements(rollers, elevator, pivot);
    }

    @Override
    public void initialize() {
        isDone = false;
        switch (location) {
            case GROUND:
                elevator.setStow();
                pivot.setGroundPickup();
                break;
            case REEFLOWER:
                isDone = !RobotObserver.getShapeChecker().apply(CommandBounds.reefBounds);
                elevator.setReefLower();
                pivot.setReefPickup();
                break;
            case REEFUPPER:
                isDone = !RobotObserver.getShapeChecker().apply(CommandBounds.reefBounds);
                elevator.setReefUpper();
                pivot.setReefPickup();
                break;
            default:
                isDone = true;
        }
    }

    @Override
    public void execute() {
        if (elevator.atSetpoint() && pivot.atSetpoint()) {
            rollers.intakeAlgae();
        }
        if (rollers.hasObject()) {
            switch (location) {
                case GROUND:
                    isDone = true;
                    break;
                case REEFLOWER:
                    isDone = !RobotObserver.getShapeChecker().apply(CommandBounds.reefBounds);
                    pivot.setReefExtract();
                    break;
                case REEFUPPER:
                    isDone = !RobotObserver.getShapeChecker().apply(CommandBounds.reefBounds);
                    pivot.setReefExtract();
                    break;
                default:
                    isDone = true;
            }
            if (drivetrain.getBluePose().getTranslation()
                    .getDistance(FieldConstants.reefCenter) >= AlgaeRollerConstants.reefPickupSafetyDistance) {
                isDone = true;
            }
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
        return isDone;
    }
}