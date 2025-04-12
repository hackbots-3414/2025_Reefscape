package frc.robot.commands;

import frc.robot.RobotObserver;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDefaultCommand extends Command {
    private final Elevator m_elevator;

    public ElevatorDefaultCommand(Elevator elevator) {
        addRequirements(elevator);
        m_elevator = elevator;
    }

    @Override
    public void execute() {
        if (m_elevator.taken()) return;
        if (RobotObserver.getReefReady()) {
            if (DriverStation.isAutonomousEnabled()) {
                m_elevator.setL4();
            } else {
                m_elevator.setPrep();
            }
        } else {
            m_elevator.setStow();
        }
        m_elevator.release();
    }
}
