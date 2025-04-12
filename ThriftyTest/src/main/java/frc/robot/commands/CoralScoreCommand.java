package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;

public class CoralScoreCommand extends Command {
    private final Logger m_logger = LoggerFactory.getLogger(CoralScoreCommand.class);

  private final CoralRollers coral;
  private final Elevator elevator;
  private final int level;

  private boolean finish = false;

  public CoralScoreCommand(CoralRollers coralRollers, Elevator elevator, int level) {
    this.coral = coralRollers;
    this.elevator = elevator;
    this.level = level;
    addRequirements(coralRollers, elevator);
  }

  @Override
  public void initialize() {
    // finish = !coral.holdingPiece();
    elevator.setLevel(level);
  }

  @Override
  public void execute() {
    if(elevator.atSetpoint()) {
      switch(level) {
        case 1 -> coral.setL1Eject();
        case 2 -> coral.setL2Eject();
        case 3 -> coral.setL3Eject();
        case 4 -> coral.setL4Eject();
        default -> m_logger.warn("invalid setpoint: {}", level);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
    elevator.release();
  }

  @Override
  public boolean isFinished() {
    return !coral.getFrontCANrange() || finish;
  }
}
