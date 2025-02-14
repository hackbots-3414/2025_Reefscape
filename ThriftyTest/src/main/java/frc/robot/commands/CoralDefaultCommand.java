package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandBounds;
import frc.robot.RobotObserver;
import frc.robot.subsystems.CoralRollers;

public class CoralDefaultCommand extends Command {
    private CoralRollers m_coral;

    public CoralDefaultCommand(CoralRollers coral, Supplier<Boolean> disableAutoCoral) {
        m_coral = coral;
        addRequirements(coral);
    }

    @Override
    public void execute() {
        if (RobotObserver.getDisableBounds() || RobotObserver.getVisionExpired()) return;

        if (RobotObserver.getShapeChecker().apply(CommandBounds.rightIntakeBounds) ||
            RobotObserver.getShapeChecker().apply(CommandBounds.leftIntakeBounds)) {
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