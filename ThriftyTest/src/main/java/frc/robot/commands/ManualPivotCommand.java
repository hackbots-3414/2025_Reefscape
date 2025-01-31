package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class ManualPivotCommand extends Command {
    private final boolean isUp;
    private final Pivot pivot;

    public ManualPivotCommand(Pivot pivot, boolean isUp) {
        this.isUp = isUp;
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        pivot.disableStateSpace();
    }

    @Override
    public void execute() {
        pivot.setSpeed(isUp ? PivotConstants.manualUpSpeed : PivotConstants.manualDownSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.enableStateSpace();
    }
}
