package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class AlgaeIntakeCommand extends Command {
  private final AlgaeRollers rollers;
  private final Elevator elevator;
  private final Pivot pivot;
  private final AlgaeLocationPresets location;
  private boolean isDone;

  public AlgaeIntakeCommand(AlgaeRollers rollers, Elevator elevator, Pivot pivot, AlgaeLocationPresets location) {
   this.rollers = rollers;
   this.elevator = elevator;
   this.pivot = pivot;
   this.location = location;
   addRequirements(rollers, elevator, pivot);
  }

  @Override
    public void initialize() {
        rollers.intakeAlgae();
        isDone = false;
        switch (location) {
            case GROUND -> {
                elevator.setGroundIntake();
                // pivot.setGroundPickup();
            }
            case REEFLOWER -> {
                isDone = !CommandBounds.reefBounds.isActive();
                elevator.setReefLower();
            }
            case REEFUPPER -> {
                isDone = !CommandBounds.reefBounds.isActive();
                elevator.setReefUpper();
            }
            default -> isDone = true;
        }
    }

    @Override
    public void execute() {
        switch (location) {
            case GROUND -> {
                if (elevator.atSetpoint()) pivot.setReefPickup();
                if (rollers.hasObject()) isDone = true; 
            }
            case REEFLOWER, REEFUPPER -> {
                isDone = !CommandBounds.reefBounds.isActive();
                if (elevator.atSetpoint()) pivot.setReefPickup();
            }
            default -> isDone = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setStow();
        rollers.smartStop();
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}