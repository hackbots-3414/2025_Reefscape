package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSetpoints;

public class ElevatorCommand extends Command {
    private Elevator m_elevator;
    private ElevatorSetpoints m_position;

    public ElevatorCommand(Elevator elevator, ElevatorSetpoints position) {
        m_elevator = elevator;
        m_position = position;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.set(m_position);
    }

    @Override
    public boolean isFinished() {
        return m_elevator.atSetpoint();
    }
}

