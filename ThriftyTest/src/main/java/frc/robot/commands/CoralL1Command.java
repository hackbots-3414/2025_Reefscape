package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;

public class CoralL1Command extends Command {
  private final CoralRollers coral;
  private final Elevator elevator;

  private int m_timeRemaining;

  public CoralL1Command(CoralRollers coralRollers, Elevator elevator) {
    this.coral = coralRollers;
    this.elevator = elevator;
    addRequirements(coralRollers, elevator);
  }

  @Override
  public void initialize() {
    m_timeRemaining = 25;
    if (!CommandBounds.reefBounds.isActive()) {
        m_timeRemaining = 0;
        return;
    }
    elevator.setL1();
  }

  @Override
  public void execute() {
    if(elevator.atSetpoint()) {
      coral.setIndividualEject();
    }
    if (!coral.presentPiece()) m_timeRemaining --;
  }

  @Override
  public void end(boolean interrupted) {
    coral.resetFollow();
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timeRemaining == 0;
  }
}
