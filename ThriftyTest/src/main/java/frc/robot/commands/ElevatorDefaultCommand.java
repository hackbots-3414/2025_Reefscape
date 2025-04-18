package frc.robot.commands;

import frc.robot.RobotObserver;
import frc.robot.subsystems.Elevator;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDefaultCommand extends Command {
    private final Logger m_logger = LoggerFactory.getLogger(ElevatorDefaultCommand.class);
    private final Elevator m_elevator;

    public ElevatorDefaultCommand(Elevator elevator) {
        addRequirements(elevator);
        m_elevator = elevator;
    }

    @Override
    public void initialize() {
        m_logger.debug("Starting");
    }

    @Override
    public void execute() {
        if (m_elevator.taken() || DriverStation.isAutonomous()) return;
        if (RobotObserver.getReefReady()) {
            if (DriverStation.isAutonomous()) {
                m_elevator.setL4();
            } else {
                m_elevator.setPrep();
            }
        } else {
            m_elevator.setStow();
        }
        m_elevator.release();
    }

    @Override
    public void end(boolean interrupted) {
        m_logger.debug("Ending");
    }
}
