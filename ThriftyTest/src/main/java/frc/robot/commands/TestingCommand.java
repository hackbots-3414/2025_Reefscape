package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TestingCommand extends Command {
    private CommandSwerveDrivetrain m_drivetrain;

    public TestingCommand(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}
}
