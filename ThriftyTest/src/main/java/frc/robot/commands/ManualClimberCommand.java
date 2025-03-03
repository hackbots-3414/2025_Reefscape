package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotObserver;
import frc.robot.subsystems.Climber;

public class ManualClimberCommand extends Command {
    private final Climber climber;
    private boolean m_up;

    public ManualClimberCommand(Climber climber) {
        this(climber, true);
    }

    public ManualClimberCommand(Climber climber, boolean goingUp) {
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
}
