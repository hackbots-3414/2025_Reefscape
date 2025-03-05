package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralRollers.CoralRollerSpeeds;

public class CoralIntakeCommand extends Command {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(CoralRollers.class);

  private final CoralRollers coral;
  private final Elevator elevator;

  private final Timer timer;

  public CoralIntakeCommand(CoralRollers coralRollers, Elevator elevator) {
    this.coral = coralRollers;
    this.elevator = elevator;
    this.timer = new Timer();
    addRequirements(coralRollers, elevator);
  }

  @Override
  public void initialize() {
    elevator.stow();
  }

  @Override
  public void execute() {
    if(elevator.atSetpoint()) {
      coral.set(CoralRollerSpeeds.INTAKE);
    }
    if (coral.holdingPiece()) timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(CoralConstants.ejectTime);
  }
}
