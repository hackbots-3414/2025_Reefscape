package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.ElevatorConstants.ScoreLevel;

public class ElevatorCommand extends Command {
    private Elevator m_elevator;
    private ElevatorPosition m_position;

    public ElevatorCommand(Elevator elevator, ElevatorPosition position) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    public ElevatorCommand(Elevator elevator, ScoreLevel level) {
        this(elevator, ElevatorPosition.fromScoreLevel(level));
    }

    @Override
    public void initialize() {
        m_elevator.setPosition(m_position);
    }

    @Override
    public boolean isFinished() {
        return m_elevator.atSetpoint();
    }
}

