package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Elevator.ElevatorSetpoints;
import frc.robot.subsystems.Pivot.PivotSetpoints;

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
                elevator.set(ElevatorSetpoints.GROUND);
            }
            case ALGAE_L2 -> {
                isDone = !CommandBounds.reefBounds.isActive();
                elevator.set(ElevatorSetpoints.ALGAE_L2);
            }
            case ALGAE_L3 -> {
                isDone = !CommandBounds.reefBounds.isActive();
                elevator.set(ElevatorSetpoints.ALGAE_L3);
            }
            case HIGHGROUND -> {
                elevator.set(ElevatorSetpoints.HIGHGROUND);
            }
            default -> isDone = true;
        }
    }

    @Override
    public void execute() {
        switch (location) {
            case GROUND, HIGHGROUND -> {
                if (elevator.atSetpoint()) pivot.set(PivotSetpoints.GROUND);
                if (rollers.hasObject()) isDone = true; 
            }
            case ALGAE_L2, ALGAE_L3 -> {
                isDone = !CommandBounds.reefBounds.isActive();
                if (elevator.atSetpoint()) {
                    if (rollers.hasObject()) {
                        pivot.set(PivotSetpoints.REEF_EXTRACT);
                    } else {
                        pivot.set(PivotSetpoints.REEF_PICKUP);
                    }
                }
            }
            default -> isDone = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stow();
        pivot.stow();
        rollers.smartStop();
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}