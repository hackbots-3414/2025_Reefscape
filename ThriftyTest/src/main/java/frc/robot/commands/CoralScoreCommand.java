package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.CoralRollers.CoralRollerSpeeds;

public class CoralScoreCommand extends Command {
    private final CoralRollers coral;
    private final CoralRollerSpeeds speed;

    public CoralScoreCommand(CoralRollers coralRollers, CoralRollerSpeeds speed) {
        this.coral = coralRollers;
        this.speed = speed;
        addRequirements(coralRollers);
    }

    @Override
    public void initialize() {
        coral.set(speed);
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
