package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotObserver;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {
    private final Climber climber;
    private boolean m_up;

    private boolean finish;

    private double initialEncoderValue;

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
        climber.setClosedLoop(false);
        initialEncoderValue = climber.getEncoderValue();
        finish = false;
        if (m_up) {
            if (climber.ready()) {
                climber.setClimbUpVolts();
            } else {
                finish = true;
            }
        } else {
            climber.setDownVolts();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotor();
        climber.closeFunnel();
        RobotObserver.setClimbed(true);
        climber.setClosedLoop(!interrupted); // only stay if we did it right.
    }

    @Override
    public boolean isFinished() {
        if (finish) return true;
        if (m_up) {
            if (Math.abs(climber.getEncoderValue() - initialEncoderValue) > ClimberConstants.climbMaxEncoderValue) {
                return true;
            }
            return climber.climbed();
        } else {
            return climber.ready();
        }
    }
}
