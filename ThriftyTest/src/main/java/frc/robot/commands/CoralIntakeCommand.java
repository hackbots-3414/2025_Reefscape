package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;

public class CoralIntakeCommand extends Command {
  private final CoralRollers coral;
  private final Elevator elevator;

  public CoralIntakeCommand(CoralRollers coralRollers, Elevator elevator) {
    this.coral = coralRollers;
    this.elevator = elevator;
    addRequirements(coralRollers, elevator);
  }

  @Override
  public void initialize() {
    elevator.setStow();
  }

  @Override
  public void execute() {
    if(elevator.atSetpoint()) {
      coral.setIntake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }
}
