package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.CoralRollers.CoralRollerSpeeds;
import frc.robot.subsystems.Elevator;

public class CoralEjectCommand extends Command {
    private CoralRollers m_coralRollers;
    private Elevator m_elevator;

  public CoralEjectCommand(CoralRollers coralRollers, Elevator elevator) {
    m_coralRollers = coralRollers;
    m_elevator = elevator;
    addRequirements(coralRollers, elevator);
  }

  @Override
  public void initialize() {
    m_elevator.stow();
  }

  @Override
  public void execute() {
    if (m_elevator.atSetpoint()) {
        m_coralRollers.set(CoralRollerSpeeds.UNJAM);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_coralRollers.stop();
    m_elevator.stow();
  }

  @Override
  public boolean isFinished() {
    return !m_coralRollers.holdingPiece();
  }
}
