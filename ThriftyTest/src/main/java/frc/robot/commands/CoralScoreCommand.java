package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;

public class CoralScoreCommand extends Command {
  private final CoralRollers coral;
  private final Elevator elevator;
  private final int level;

  private boolean isDone;

  public CoralScoreCommand(CoralRollers coralRollers, Elevator elevator, int level) {
    this.coral = coralRollers;
    this.elevator = elevator;
    this.level = level;
    addRequirements(coralRollers, elevator);
    
    SmartDashboard.putBoolean("SCORING L1", false);
  }

  @Override
  public void initialize() {
    // if (!CommandBounds.reefBounds.isActive()) {
    //     isDone = true;
    //     return;
    // }
    switch(level) {
      case 1 -> {
        elevator.setL1();
        SmartDashboard.putBoolean("SCORING L1", true);
        break;
      }
      case 2 -> elevator.setL2();
      case 3 -> elevator.setL3();
      case 4 -> elevator.setL4();
      default -> isDone = true;
    }
  }

  @Override
  public void execute() {
    if(elevator.atSetpoint()) {
      coral.setEject();
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setStow();
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return !coral.presentPiece() || isDone;
  }
}
