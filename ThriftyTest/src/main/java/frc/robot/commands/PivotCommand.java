package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotPosition;

public class PivotCommand extends Command {
    private Pivot m_pivot;
    private PivotPosition m_position;

    public PivotCommand(Pivot pivot, PivotPosition position) {
        m_pivot = pivot;
        m_position = position;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {
        m_pivot.setPosition(m_position);
    }

    @Override
    public boolean isFinished() {
        return m_pivot.atSetpoint();
    }
}
