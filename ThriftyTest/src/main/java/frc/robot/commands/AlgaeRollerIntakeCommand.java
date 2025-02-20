package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRollers;

public class AlgaeRollerIntakeCommand extends Command {
    private AlgaeRollers m_rollers;

    public AlgaeRollerIntakeCommand(AlgaeRollers rollers) {
        m_rollers = rollers;
        addRequirements(m_rollers);
    }

    @Override
    public void execute() {
        m_rollers.intakeAlgae();
    }

    @Override
    public boolean isFinished() {
        return m_rollers.hasObject();
    }
}
