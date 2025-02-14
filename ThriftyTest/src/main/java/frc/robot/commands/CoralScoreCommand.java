package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.RobotObserver;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;

public class CoralScoreCommand extends Command {
  private CoralRollers coral;
  private Elevator elevator;
  private int level;

  private boolean isDone;

  public CoralScoreCommand(CoralRollers coralRollers, Elevator elevator, int level) {
    this.coral = coralRollers;
    this.elevator = elevator;
    this.level = level;
    addRequirements(coralRollers, elevator);
  }

  @Override
  public void initialize() {
    if (!RobotObserver.getShapeChecker().apply(CommandBounds.reefBounds)) {
        isDone = true;
        return;
    }
    switch(level) {
      case 1:
        elevator.setL1();
        break;

      case 2:
        elevator.setL2();
        break;
      
      case 3:
        elevator.setL3();
        break;
      
      case 4:
        elevator.setL4();
        break;
      
      default:
        isDone = true;
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
    return !coral.holdingPiece() || isDone;
  }
}
