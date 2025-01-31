package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ManualClimberCommand extends Command {
    private Climber climber;

    public ManualClimberCommand(Climber climber) {
        addRequirements(climber);
        this.climber = climber;
    }

    @Override
    public void initialize() {
            climber.setClimbUpVolts();

    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotor();
    }
}
