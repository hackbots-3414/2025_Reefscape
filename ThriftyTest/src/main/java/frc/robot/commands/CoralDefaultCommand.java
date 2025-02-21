package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.RobotObserver;
import frc.robot.subsystems.CoralRollers;

public class CoralDefaultCommand extends Command {
    private final CoralRollers m_coral;

    public CoralDefaultCommand(CoralRollers coral) {
        m_coral = coral;
        addRequirements(coral);
    }

    @Override
    public void execute() {
        if (!RobotObserver.getVisionValid()) return;

        if (CommandBounds.rightIntakeBounds.isActive() ||
            CommandBounds.leftIntakeBounds.isActive()) {
            m_coral.resetTimeout();
            m_coral.setIntake();
        } else {
            m_coral.timeoutIntake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_coral.resetTimeout();
    }
}