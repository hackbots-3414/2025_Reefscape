package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRollers;

public class AlgaeRollerCommand extends Command {
    private final Logger m_logger = LoggerFactory.getLogger(AlgaeRollerCommand.class);

    private AlgaeRollers rollers;

    public AlgaeRollerCommand(AlgaeRollers rollers) {
        addRequirements(rollers);
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        m_logger.warn("Not yet sure how we want to handle the logic for algae roller intake vs eject. Figure out, then implement it here.");
        rollers.intakeAlgae();
    }

    @Override
    public void execute() {
        rollers.intakeAlgae();
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stopMotor();
    }
}
