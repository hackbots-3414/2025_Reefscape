package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class CoralEjectCommand extends Command {
  private final Superstructure superstructure;

  public CoralEjectCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.intake();
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.stow();
  }

  @Override
  public boolean isFinished() {
    return !superstructure.hasCoral();
  }
}
