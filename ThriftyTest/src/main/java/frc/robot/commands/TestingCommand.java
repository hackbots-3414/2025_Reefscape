package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TestingCommand extends Command {
    private CommandSwerveDrivetrain m_drivetrain;
    private SwerveRequest m_request = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withVelocityX(2.0);

    public TestingCommand(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.setControl(m_request);
    }

    @Override
    public void end(boolean interruped) {
        m_drivetrain.stop();
    }

}
