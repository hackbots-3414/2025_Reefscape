package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class CoralScoreCommand extends Command {
  private Coral coral;
  private Elevator elevator;
  private int Level;
  private boolean setEjectAlready;

  public CoralScoreCommand( Coral coral, Elevator elevator, int Level) {
    this.coral = coral;
    this.elevator = elevator;
    this.Level = Level;
  }

  
  @Override
  public void initialize() {
    switch(Level) {
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
        elevator.setStow();
    }
  }

  @Override
  public void execute() {
    if(elevator.atSetpoint()) {
      if(!setEjectAlready) {
        coral.setEject();
        setEjectAlready = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setStow();
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return !coral.holdingPiece();
  }
}
