package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotObserver;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldUtils;

public class DriveToPointCommand extends Command {
    private final Logger m_logger = LoggerFactory.getLogger(DriveToPointCommand.class);

    private final ProfiledPIDController xController = new ProfiledPIDController(
            DriveConstants.k_driveToPointTranslationPID.kP, 0, 0, DriveConstants.k_driveToPointConstraints);
    private final ProfiledPIDController yController = new ProfiledPIDController(
            DriveConstants.k_driveToPointTranslationPID.kP, 0, 0, DriveConstants.k_driveToPointConstraints);

    private static Rotation2d m_targetRotation;

    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withHeadingPID(DriveConstants.k_driveToPointRotationPID.kP, 0, 0)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDriveRequestType(DriveRequestType.Velocity);

    private Pose2d m_goal;
    private final CommandSwerveDrivetrain m_drivetrain;

    private boolean m_flip;

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
        // flip goal if necessary
        if (m_flip) {
            m_goal = FieldUtils.flipPose(m_goal);
            m_flip = false;
        }
        m_targetRotation = m_goal.getRotation();

        Pose2d currPose = m_drivetrain.getPose();
        Translation2d currVelo = m_drivetrain.getVelocityComponents();

        // so first is finished run doesn't break
        xController.reset(currPose.getX(), currVelo.getX());
        yController.reset(currPose.getY(), currVelo.getY());

        xController.setGoal(m_goal.getX());
        yController.setGoal(m_goal.getY());

        xController.calculate(currPose.getX());
        yController.calculate(currPose.getY());

        RobotObserver.getField().getObject("target").setPose(m_goal);
    }

    @Override
    public void execute() {
        Pose2d pose = m_drivetrain.getPose();
        double vx = xController.calculate(pose.getX());
        double vy = yController.calculate(pose.getY());

        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);

        m_drivetrain.setControl(
                m_request
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(m_targetRotation));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
        m_logger.debug("drive to pose interrupted: {}", interrupted);
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

        m_logger.debug("err: {}, errRotation: {}", err, errRotation);

        return (err < AutonConstants.translationTolerance && errRotation < AutonConstants.rotationTolerance.in(Radians));
    }
}
