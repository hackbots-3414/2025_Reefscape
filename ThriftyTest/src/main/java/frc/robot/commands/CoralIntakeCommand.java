package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;

public class CoralIntakeCommand extends Command {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(CoralRollers.class);

  private final CoralRollers coral;
  private final Elevator elevator;

  private boolean pulledForward = false;

  public CoralIntakeCommand(CoralRollers coralRollers, Elevator elevator) {
    this.coral = coralRollers;
    this.elevator = elevator;
    addRequirements(coralRollers, elevator);
  }

  @Override
  public void initialize() {
    pulledForward = false;
    if (coral.holdingPiece()) pulledForward = true; // don't run again.
    elevator.setStow();
  }

  @Override
  public void execute() {
    if(elevator.atSetpoint()) {
      if (coral.onlyFrontIR()) {
        pulledForward = true;
        coral.setRetract();
      } else {
        if (!pulledForward) coral.setIntake();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return coral.holdingPiece() && pulledForward;
  }
}
