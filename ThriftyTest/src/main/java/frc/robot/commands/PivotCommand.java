package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.subsystems.Pivot;

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
        switch (m_position) {
            case Stow -> m_pivot.setStow();
            case Net -> m_pivot.setNet();
            case ReefPickup -> m_pivot.setReefPickup();
            case ReefExtract -> m_pivot.setReefExtract();
            case Ground -> m_pivot.setGroundPickup();
            case Processor -> m_pivot.setProcessor();
        }
    }

    @Override
    public boolean isFinished() {
        return m_pivot.atSetpoint();
    }

    public enum PivotPosition {
        Stow,
        Net,
        ReefPickup,
        ReefExtract,
        Ground,
        Processor;

        public static PivotPosition fromAlgaePreset(
            AlgaeLocationPresets preset
        ) {
            PivotPosition position = null;
            switch (preset) {
                case REEFUPPER,REEFLOWER -> position = ReefPickup;
                case PROCESSOR -> position = Processor;
                case NET -> position = Net;
                case GROUND -> position = Ground;
            }
            return position;
        }
    }
}
