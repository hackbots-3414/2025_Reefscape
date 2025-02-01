package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

public class CoralEjectCommand extends Command {
  private Coral coral;

  public CoralEjectCommand(Coral coral) {
      this.coral = coral;
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
    return !coral.holdingPiece();
  }
}
