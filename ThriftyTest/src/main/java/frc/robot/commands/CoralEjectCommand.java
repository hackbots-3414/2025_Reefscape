package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollers;

public class CoralEjectCommand extends Command {
  private final CoralRollers coral;

  public CoralEjectCommand(CoralRollers coralRollers) {
    this.coral = coralRollers;
    addRequirements(coralRollers);
  }

  @Override
  public void initialize() {
    coral.setEject();
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  @Override
  public boolean isFinished() {
    return !coral.presentPiece();
  }
}
