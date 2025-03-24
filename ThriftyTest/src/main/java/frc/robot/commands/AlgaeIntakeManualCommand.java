package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRollers;

public class AlgaeIntakeManualCommand extends Command {
  private final AlgaeRollers algae;

  public AlgaeIntakeManualCommand(AlgaeRollers algae) {
    this.algae = algae;
  }

  @Override
  public void initialize() {
    algae.intakeAlgae();
  }

  @Override
  public void end(boolean interrupted) {
    algae.smartStop();
  }

  @Override
  public boolean isFinished() {
    return algae.algaeHeld();
  }
}
