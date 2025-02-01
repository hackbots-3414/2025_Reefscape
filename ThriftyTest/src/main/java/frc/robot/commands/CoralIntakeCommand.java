package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class CoralIntakeCommand extends Command {
  private Coral coral;
  private Elevator elevator;

  private boolean setIntakeAlready;

  public CoralIntakeCommand(Coral coral,Elevator elevator) {
    this.coral = coral;
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    elevator.setStow();
  }

  @Override
  public void execute() {
    if(elevator.atSetpoint()) {
      if(!setIntakeAlready) {
        coral.setIntake();
        setIntakeAlready = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return coral.holdingPiece();
  }
}
