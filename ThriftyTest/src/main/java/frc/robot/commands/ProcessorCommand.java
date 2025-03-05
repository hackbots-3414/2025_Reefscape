package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Elevator.ElevatorSetpoints;
import frc.robot.subsystems.Pivot.PivotSetpoints;

public class ProcessorCommand extends Command {
    private Elevator m_elevator;
    private AlgaeRollers m_rollers;
    private Pivot m_pivot;

    public ProcessorCommand(Elevator elevator, AlgaeRollers rollers, Pivot pivot) {
        m_elevator = elevator;
        m_rollers = rollers;
        m_pivot = pivot;
        addRequirements(elevator, rollers, pivot);
    }

    @Override
    public void initialize() {
        m_elevator.set(ElevatorSetpoints.PROCESSOR);
        m_pivot.set(PivotSetpoints.PROCESSOR);
    }

    @Override
    public void execute() {
        if (m_elevator.atSetpoint() && m_pivot.atSetpoint()) {
            m_rollers.eject();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_rollers.smartStop();
    }
}
