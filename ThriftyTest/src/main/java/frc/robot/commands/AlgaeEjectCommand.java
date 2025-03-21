package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Elevator;

public class AlgaeEjectCommand extends Command {
  private final AlgaeRollers m_rollers;
  private final Elevator m_elevator;
  
  public AlgaeEjectCommand(AlgaeRollers rollers, Elevator elevator) {
    m_elevator = elevator;
    m_rollers = rollers;
    addRequirements(rollers);
  }

    @Override
    public void initialize() {
      m_rollers.processorEjectAlgae();
      m_elevator.setProcessor();
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}