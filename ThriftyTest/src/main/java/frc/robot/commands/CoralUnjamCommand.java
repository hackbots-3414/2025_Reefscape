package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollers;

public class CoralUnjamCommand extends Command {
    private final CoralRollers coral;

    public CoralUnjamCommand(CoralRollers coralRollers) {
        this.coral = coralRollers;
        addRequirements(coralRollers);
    }

    @Override
    public void initialize() {
        coral.unjam();
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
