package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {
    private Elevator m_elevator;
    private ElevatorPosition m_position;

    public ElevatorCommand(Elevator elevator, ElevatorPosition position) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        switch (m_position) {
            case L1 -> m_elevator.setL1();
            case L2 -> m_elevator.setL2();
            case L3 -> m_elevator.setL3();
            case L4 -> m_elevator.setL4();
            case Net -> m_elevator.setNet();
            case Stow -> m_elevator.setStow();
            case Ground -> m_elevator.setGround();
            case ReefHigh -> m_elevator.setReefLower();
            case ReefLow -> m_elevator.setReefUpper();
            case Processor -> m_elevator.setProcessor();
        }
    }

    @Override
    public boolean isFinished() {
        return m_elevator.atSetpoint();
    }

    public enum ElevatorPosition {
        L1,
        L2,
        L3,
        L4,
        Net,
        Stow,
        Ground,
        ReefHigh,
        ReefLow,
        Processor;

        public static ElevatorPosition fromAlgaePreset(
            AlgaeLocationPresets preset
        ) {
            ElevatorPosition position = null;
            switch (preset) {
                case REEFUPPER -> position = ReefHigh;
                case REEFLOWER -> position = ReefLow;
                case PROCESSOR -> position = Processor;
                case NET -> position = Net;
                case GROUND -> position = Ground;
            }
            return position;
        }
    }
}

