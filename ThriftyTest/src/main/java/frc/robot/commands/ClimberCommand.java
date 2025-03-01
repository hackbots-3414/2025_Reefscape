package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotObserver;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
    private final Climber climber;
    private boolean m_up;

    public ClimberCommand(Climber climber) {
        this(climber, true);
    }

    public ClimberCommand(Climber climber, boolean goingUp) {
        addRequirements(climber);
        this.climber = climber;
        m_up = goingUp;
    }

    @Override
    public void initialize() {
        if (m_up) {
            climber.setClimbUpVolts();
        } else {
            climber.setDownVolts();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotor();
        RobotObserver.setClimbed(true);
    }

    @Override
    public boolean isFinished() {
        if (m_up) {
            return climber.climbed();
        } else {
            return climber.ready();
        }
    }
}
