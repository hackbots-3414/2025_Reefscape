package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeRollerConstants;
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

  public AlgaeIntakeCommand(AlgaeRollers rollers, Elevator elevator, Pivot pivot, CommandSwerveDrivetrain drivetrain, AlgaeLocationPresets location) {
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
        elevator.setReefLower();
        pivot.setReefPickup();
        break;
      case REEFUPPER:
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
          pivot.setReefExtract();
          break;
        case REEFUPPER:
          pivot.setReefExtract();
          break;       
        default:
          isDone = true;
      }
      if (drivetrain.getFlippedPose().getTranslation().getDistance(FieldConstants.reefCenter) >= AlgaeRollerConstants.reefPickupSafetyDistance) {
        isDone = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setStow();
    pivot.setStow();
  }

  @Override
  public boolean isFinished() {
   return isDone;  
  }
}