package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRollers;

public class AlgaeEjectCommand extends Command {
  private final AlgaeRollers rollers;
  
  public AlgaeEjectCommand(AlgaeRollers rollers) {
   this.rollers = rollers;
   addRequirements(rollers);
  }

  @Override
  public void initialize() {
    rollers.ejectAlgae();
  }

  @Override
  public void end(boolean interrupted) {
    rollers.stopMotor();
  }
}