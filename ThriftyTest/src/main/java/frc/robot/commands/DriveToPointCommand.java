package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.driveassist.Autopilot;
import frc.robot.RobotObserver;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldUtils;

public class DriveToPointCommand extends Command {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(DriveToPointCommand.class);

    private static Rotation2d m_targetRotation;

    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withHeadingPID(DriveConstants.k_driveToPointRotationPID.kP, 0, 0)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.Velocity);

    private Pose2d m_goal;
    private final CommandSwerveDrivetrain m_drivetrain;

    private boolean m_flip;

    private List<Pose2d> m_poses = new ArrayList<>();

    private Autopilot m_autopilot = new Autopilot(DriveConstants.kAutopilotConstraints);

    public DriveToPointCommand(Pose2d pose, CommandSwerveDrivetrain drivetrain) {
        this(pose, drivetrain, false);
    }

    public DriveToPointCommand(Pose2d pose, CommandSwerveDrivetrain drivetrain, boolean flipPose) {
        m_goal = pose;
        m_drivetrain = drivetrain;
        m_flip = flipPose;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.setAligned(false);
        // flip goal if necessary
        if (m_flip) {
            m_goal = FieldUtils.flipPose(m_goal);
            m_flip = false;
        }
        m_targetRotation = m_goal.getRotation();

        RobotObserver.getField().getObject("target").setPose(m_goal);
        m_poses.clear();
    }

    @Override
    public void execute() {
        Pose2d pose = m_drivetrain.getPose();
        m_poses.add(pose);
        RobotObserver.getField().getObject("path").setPoses(m_poses);
        Translation2d velo = m_drivetrain.getVelocityComponents();
        Translation2d adjusted = m_autopilot.adjust(pose, m_goal, velo);
        m_drivetrain.setControl(m_request
            .withVelocityX(adjusted.getX())
            .withVelocityY(adjusted.getY())
            .withTargetDirection(m_targetRotation)
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
        double errX = m_drivetrain.getPose().getX() - m_goal.getX();
        double errY = m_drivetrain.getPose().getY() - m_goal.getY();
        double err = Math.hypot(errX, errY);

        double errRotation = Math.abs(m_drivetrain.getPose()
                .getRotation()
                .minus(m_targetRotation)
                .getRadians());

        return (err < AutonConstants.translationTolerance && errRotation < AutonConstants.rotationTolerance.in(Radians));
    }
}
