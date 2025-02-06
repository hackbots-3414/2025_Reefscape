package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class AlgaeScoreCommand extends Command {
  private AlgaeRollers rollers;
  private Elevator elevator;
  private Pivot pivot;
  private AlgaeLocationPresets location;
  private boolean isDone;
  private double timeElapsed;

  public AlgaeScoreCommand(AlgaeRollers rollers, Elevator elevator, Pivot pivot, AlgaeLocationPresets location) {
   this.rollers = rollers;
   this.elevator = elevator;
   this.pivot = pivot;
   this.location = location;
   addRequirements(rollers); // don't add requirements on elevator and pivot; statespace will control them
  }

  @Override
  public void initialize() {
    timeElapsed = 0;
    isDone = false;
    switch (location) {
      case NET: 
        elevator.setNet();
        pivot.setNet();
        break;
      case PROCESSOR:
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
      timeElapsed += 0.02;
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
    return isDone || timeElapsed >= AlgaeRollerConstants.algaeEjectTime;  
  }
}