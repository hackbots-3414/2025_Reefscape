package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.RobotObserver;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;

public class CoralDefaultCommand extends Command {
    private final CoralRollers m_coral;
    private final Elevator m_elevator;

    private int timeRemaining = 3;

    public CoralDefaultCommand(CoralRollers coral, Elevator elevator) {
        m_coral = coral;
        m_elevator = elevator;
        addRequirements(coral);
    }

    @Override
    public void execute() {
        if (!RobotObserver.getVisionValid()) return;

        if (CommandBounds.rightIntakeBounds.isActive() ||
            CommandBounds.leftIntakeBounds.isActive()) {
            m_coral.resetTimeout();

            if (m_coral.holdingPiece()) {
                timeRemaining--;
            } else {
                timeRemaining = 3;
            }

            if (timeRemaining <= 0) {
                m_coral.stop();
            } else {
                // **************************** MUST BE ROBOT OBSERVER
                if (m_elevator.atSetpoint()) {
                    m_coral.setIntake();
                }
            }
        } else {
            m_coral.timeoutIntake();
            timeRemaining = 3;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_coral.resetTimeout();
    }
}