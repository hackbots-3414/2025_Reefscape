package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRollers;

public class AlgaeEjectCommand extends Command {
  private final AlgaeRollers m_rollers;
  private final boolean m_stop;
  
  public AlgaeEjectCommand(AlgaeRollers rollers) {
    this(rollers, false);
  }
  
  public AlgaeEjectCommand(AlgaeRollers rollers, boolean stop) {
    m_rollers = rollers;
    m_stop = stop;
    addRequirements(rollers);
  }

  @Override
  public void initialize() {
    m_rollers.ejectAlgae();
  }

  @Override
  public void end(boolean interrupted) {
    m_rollers.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return (m_stop && !m_rollers.hasObject());
  }
}