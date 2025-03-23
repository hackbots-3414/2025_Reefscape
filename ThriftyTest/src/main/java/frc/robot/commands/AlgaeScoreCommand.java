package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.Constants.CommandBounds;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class AlgaeScoreCommand extends Command {
  private final AlgaeRollers rollers;
  private final Elevator elevator;
  private final Pivot pivot;
  private final AlgaeLocationPresets location;
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
    isDone = false;
    switch (location) {
      case NET -> {
        isDone = !CommandBounds.netBounds.isActive();
        elevator.setNet();
        pivot.setNet();
      }
      case PROCESSOR -> {
        // isDone = !CommandBounds.processorBounds.isActive();
        elevator.setProcessor();
        pivot.setProcessor();
      }
      default -> isDone = true;
    }
  }

  @Override
  public void execute() {
    if (elevator.atSetpoint() && pivot.atSetpoint()) {
      if (location == AlgaeLocationPresets.NET) {
        rollers.ejectAlgae();
      }
    } else {
      initialTime = Utils.getCurrentTimeSeconds();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (location != AlgaeLocationPresets.PROCESSOR) elevator.setStow();
    if (location != AlgaeLocationPresets.PROCESSOR) pivot.setStow();
    rollers.smartStop();
  }

  @Override
  public boolean isFinished() {
    return isDone || (Utils.getCurrentTimeSeconds() - initialTime) >= AlgaeRollerConstants.algaeEjectTime;
  }
}
