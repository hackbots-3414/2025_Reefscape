package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralRollers.CoralRollerSpeeds;

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
      case 1, 2, 3, 4 -> elevator.setLevel(level);
      default -> m_timeRemaining = 0;
    }
  }

  @Override
  public void execute() {
    if(elevator.atSetpoint()) {
      switch(level) {
        case 1 -> coral.set(CoralRollerSpeeds.L1);
        case 2 -> coral.set(CoralRollerSpeeds.L2);
        case 3 -> coral.set(CoralRollerSpeeds.L3);
        case 4 -> coral.set(CoralRollerSpeeds.L4);
        default -> m_timeRemaining = 0;
      }
    }
    if (!coral.presentPiece()) m_timeRemaining --;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stow();
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timeRemaining == 0;
  }
}
