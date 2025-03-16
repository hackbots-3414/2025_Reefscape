package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotObserver;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
    private final Climber climber;
    private boolean m_climbing;

    public ClimberCommand(Climber climber) {
        this(climber, true);
    }

    public ClimberCommand(Climber climber, boolean climbing) {
        addRequirements(climber);
        this.climber = climber;
        m_climbing = climbing;
    }

    @Override
    public void initialize() {
        if (m_climbing) {
            climber.setDown();
        } else {
            climber.setUp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotor();
        climber.closeFunnel();
        RobotObserver.setClimbed(true);
    }

    @Override
    public boolean isFinished() {
        if (m_climbing) {
            return climber.atClimb();
        } else {
            return climber.climbReady();
        }
    }
}
