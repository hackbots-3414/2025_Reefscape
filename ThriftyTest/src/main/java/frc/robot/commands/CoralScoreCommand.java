package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;

public class CoralScoreCommand extends Command {
    private final Logger m_logger = LoggerFactory.getLogger(CoralScoreCommand.class);

  private final CoralRollers coral;
  private final Elevator elevator;
  private final int level;

  private boolean finish = false;

  private int m_timeRemaining;

  public CoralScoreCommand(CoralRollers coralRollers, Elevator elevator, int level) {
    this.coral = coralRollers;
    this.elevator = elevator;
    this.level = level;
    addRequirements(coralRollers, elevator);
  }

  @Override
  public void initialize() {
    if (!coral.presentPiece()) {
      finish = true;
      return;
    }
        
    m_timeRemaining = 12;
    if (!CommandBounds.reefBounds.isActive()) {
        m_timeRemaining = 0;
        return;
    }

    finish = false;
  }

  @Override
  public void execute() {
    if(elevator.atSetpoint()) {
      switch(level) {
        case 1 -> coral.setL1Eject();
        case 2 -> coral.setL2Eject();
        case 3 -> coral.setL3Eject();
        case 4 -> coral.setL4Eject();
        default -> m_timeRemaining = 0;
      }
    }
    if (!coral.onlyFrontIR()) m_timeRemaining --;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_logger.warn("Elevator Reference: {}, Elevator Position: {}", elevator.getReference(), elevator.getPosition());
    }
    elevator.setStow();
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    if (finish) return true;
    
    if (!elevator.atSetpoint()) return true;

    if (elevator.getReference() == ElevatorConstants.stow) return true;

    return m_timeRemaining == 0;
  }
}
