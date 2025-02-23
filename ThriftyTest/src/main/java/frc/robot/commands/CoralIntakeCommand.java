package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollers;

public class CoralIntakeCommand extends Command {
  private final CoralRollers coral;

  public CoralIntakeCommand(CoralRollers coralRollers) {
    this.coral = coralRollers;
    addRequirements(coralRollers);
  }

  @Override
  public void initialize() {
      coral.setIntake();
  }

  @Override
  public void execute() {
      coral.setIntake();
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return coral.holdingPiece();
  }
}
