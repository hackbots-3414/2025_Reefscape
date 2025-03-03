package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class PitClimbSetupCommand extends Command {
    private final Climber m_climber;

    public PitClimbSetupCommand(Climber climber) {
        m_climber = climber;
    }

    @Override
    public void initialize() {
        m_climber.setClimbRoll();
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stopMotor();
    }
}
