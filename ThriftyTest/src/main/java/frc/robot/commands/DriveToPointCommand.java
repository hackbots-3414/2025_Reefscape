package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutopilotConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.driveassist.APTarget;
import frc.robot.RobotObserver;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPointCommand extends Command {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(DriveToPointCommand.class);

    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withHeadingPID(DriveConstants.k_driveToPointRotationPID.kP, 0, 0)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.Velocity);

    private boolean m_allianceRelative;

    private APTarget m_target;
    private final CommandSwerveDrivetrain m_drivetrain;

    public DriveToPointCommand(APTarget target, CommandSwerveDrivetrain drivetrain, boolean allianceRelative) {
        m_target = target;
        m_allianceRelative = allianceRelative;
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public DriveToPointCommand(Pose2d goal, CommandSwerveDrivetrain drivetrain, boolean allianceRelative) {
        this(new APTarget(goal).withEntryAngle(goal.getRotation()), drivetrain, allianceRelative);
    }

    public DriveToPointCommand(APTarget target, CommandSwerveDrivetrain drivetrain) {
        this(target, drivetrain, false);
    }

    public DriveToPointCommand(Pose2d goal, CommandSwerveDrivetrain drivetrain) {
        this(goal, drivetrain, false);
    }

    @Override
    public void initialize() {
        m_drivetrain.setAligned(false);
        if (m_allianceRelative && DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
            m_target = m_target.flip();
            m_allianceRelative = false;
        }
    }

    @Override
    public void execute() {
        // This uses the tight profile
        Translation2d adjusted = AutopilotConstants.kTightAutopilot.calculate(m_drivetrain.getPose(), m_drivetrain.getVelocityComponents(), m_target);

        m_drivetrain.setControl(m_request
            .withVelocityX(adjusted.getX())
            .withVelocityY(adjusted.getY())
            .withTargetDirection(m_target.getReference().getRotation())
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setAligned(!interrupted);
        m_drivetrain.stop();
        RobotObserver.getField().getObject("target").setPoses();
    }

    @Override
    public boolean isFinished() {
        return AutopilotConstants.kTightAutopilot.atSetpoint(m_drivetrain.getPose(), m_target.getReference());
    }
}
