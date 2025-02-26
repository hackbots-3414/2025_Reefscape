package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;

public class CoralScoreCommand extends Command {
  private final CoralRollers coral;
  private final Elevator elevator;
  private final int level;

  private int m_timeRemaining;

  public CoralScoreCommand(CoralRollers coralRollers, Elevator elevator, int level) {
    this.coral = coralRollers;
    this.elevator = elevator;
    this.level = level;
    addRequirements(coralRollers, elevator);
  }

  @Override
  public void initialize() {
    m_timeRemaining = 25;
    if (!CommandBounds.reefBounds.isActive()) {
        m_timeRemaining = 0;
        return;
    }
    switch(level) {
      case 1 -> elevator.setL1();
      case 2 -> elevator.setL2();
      case 3 -> elevator.setL3();
      case 4 -> elevator.setL4();
      default -> m_timeRemaining = 0;
    }
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
    if (!coral.presentPiece()) m_timeRemaining --;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setStow();
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timeRemaining == 0;
  }
}
