package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRollers;

public class AlgaeEjectCommand extends Command {
  private final AlgaeRollers m_rollers;
  
  public AlgaeEjectCommand(AlgaeRollers rollers) {
    m_rollers = rollers;
    addRequirements(rollers);
  }

    @Override
    public void initialize() {
      m_rollers.processorEjectAlgae();
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}