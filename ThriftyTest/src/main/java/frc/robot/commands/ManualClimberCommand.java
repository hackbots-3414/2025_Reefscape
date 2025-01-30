package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
    Climber climber;
    boolean goingUp;

    public ClimberCommand(Climber climber, boolean goingUp) {
        addRequirements(climber);
        this.climber = climber;
        this.goingUp = goingUp;
    }

    @Override
    public void execute() {
        if (goingUp) {
            climber.setClimbUpVolts();
        } else {
            climber.stopMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotor();
    }

    public static Command makeClimberCommand(boolean goingUp) {
        return new ClimberCommand(climber, goingUp);
    }
}
