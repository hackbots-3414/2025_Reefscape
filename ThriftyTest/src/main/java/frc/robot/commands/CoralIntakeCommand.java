package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.CoralRollers.CoralRollerSpeeds;

public class CoralIntakeCommand extends Command {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(CoralRollers.class);

    private final CoralRollers coral;

    private int timeRemaining = 3;

    public CoralIntakeCommand(CoralRollers coralRollers) {
        this.coral = coralRollers;
        addRequirements(coralRollers);
    }

    @Override
    public void initialize() {
        coral.set(CoralRollerSpeeds.INTAKE);
        timeRemaining = 3;
    }

    @Override
    public void execute() {
        coral.set(CoralRollerSpeeds.INTAKE);
        if (coral.holdingPiece())
            timeRemaining--;
    }

    @Override
    public void end(boolean interrupted) {
        coral.stop();
    }

    @Override
    public boolean isFinished() {
        return timeRemaining == 0;
    }
}
