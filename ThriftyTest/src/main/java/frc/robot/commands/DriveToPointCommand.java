package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    private Autopilot.Target m_target;
    private final CommandSwerveDrivetrain m_drivetrain;

    private boolean m_flip;

    private List<Pose2d> m_oldPoses = new ArrayList<>();

    private Autopilot m_autopilot = DriveConstants.kAutopilot;

    public DriveToPointCommand(Autopilot.Target target, CommandSwerveDrivetrain drivetrain, boolean flipPose) {
        m_target = target;
        m_drivetrain = drivetrain;
        m_flip = flipPose;
        addRequirements(drivetrain);
    }

    public DriveToPointCommand(Autopilot.Target target, CommandSwerveDrivetrain drivetrain) {
        this(target, drivetrain, false);
    }

    public DriveToPointCommand(Pose2d reference, CommandSwerveDrivetrain drivetrain, boolean flipPose) {
        this(new Autopilot.Target(reference, reference.getRotation()), drivetrain, flipPose);
    }

    public DriveToPointCommand(Pose2d reference, CommandSwerveDrivetrain drivetrain) {
        this(reference, drivetrain, false);
    }

    @Override
    public void initialize() {
        m_drivetrain.setAligned(false);
        // flip goal if necessary
        if (m_flip) {
            m_target = FieldUtils.flipPose(m_target);
            m_flip = false;
        }
        m_targetRotation = m_target.getReference().getRotation();

        RobotObserver.getField().getObject("target").setPose(m_target.getReference());
        m_oldPoses.clear();
    }

    @Override
    public void execute() {
        /* real */
        Pose2d pose = m_drivetrain.getPose();
        m_oldPoses.add(pose);
        RobotObserver.getField().getObject("past").setPoses(m_oldPoses);
        Translation2d velo = m_drivetrain.getVelocityComponents();
        Translation2d adjusted = m_autopilot.adjust(pose, velo, m_target);
        m_drivetrain.setControl(m_request
            .withVelocityX(adjusted.getX())
            .withVelocityY(adjusted.getY())
            .withTargetDirection(m_targetRotation)
        );
        path(pose, velo, "ideal");
    }

    private void path(Pose2d pose, Translation2d velo, String name) {
        pose = new Pose2d(pose.getTranslation(), Rotation2d.kZero);
        List<Pose2d> poses = new ArrayList<>();
        int i = 0;
        while (!atSetpoint(pose) && i++ < 300) {
            Translation2d adjusted = m_autopilot.adjust(pose, velo, m_target);
            pose = pose.plus(new Transform2d(adjusted.times(0.02), Rotation2d.kZero));
            velo = adjusted;
            poses.add(pose);
        }
        RobotObserver.getField().getObject(name).setPoses(poses);
    }

    private boolean atSetpoint(Pose2d pose) {
        double errX = pose.getX() - m_target.getReference().getX();
        double errY = pose.getY() - m_target.getReference().getY();
        double err = Math.hypot(errX, errY);

        return err < AutonConstants.translationTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setAligned(!interrupted);
        m_drivetrain.stop();
        RobotObserver.getField().getObject("target").setPoses();
        RobotObserver.getField().getObject("ideal").setPoses();
        RobotObserver.getField().getObject("past").setPoses();
        m_oldPoses.clear();
    }

    @Override
    public boolean isFinished() {

        double errRotation = Math.abs(m_drivetrain.getPose()
                .getRotation()
                .minus(m_targetRotation)
                .getRadians());

        return atSetpoint(m_drivetrain.getPose()) && errRotation < AutonConstants.rotationTolerance.in(Radians);
    }
}
