package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotObserver;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPointCommand extends Command {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(DriveToPointCommand.class);

    private final Constraints constraints = new Constraints(DriveConstants.k_driveToPointSpeed, DriveConstants.k_driveToPointAcceleration);

    private final ProfiledPIDController xPIDController = new ProfiledPIDController(DriveConstants.k_translationPID.kP, 0, 0, constraints);
    private final ProfiledPIDController yPIDController = new ProfiledPIDController(DriveConstants.k_translationPID.kP, 0, 0, constraints);

    private static Rotation2d m_targetRotation;

    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withHeadingPID(DriveConstants.k_rotationPID.kP, 0, 0)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final Pose2d m_goal;
    private final CommandSwerveDrivetrain m_drivetrain;

    public DriveToPointCommand(Pose2d pose, CommandSwerveDrivetrain drivetrain) {
        this.m_goal = pose;
        this.m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_targetRotation = m_goal.getRotation();

        Pose2d currPose = m_drivetrain.getPose();

        // so first is finished run doesn't break
        xPIDController.reset(currPose.getX());
        yPIDController.reset(currPose.getY());

        RobotObserver.getField().getObject("target").setPose(m_goal);
    }

    @Override
    public void execute() {
        Pose2d currPose = m_drivetrain.getPose();

        double xVelo = xPIDController.calculate(currPose.getX(), m_goal.getX());
        double yVelo = yPIDController.calculate(currPose.getY(), m_goal.getY());

        SmartDashboard.putNumber("XVELO", xVelo);
        SmartDashboard.putNumber("YVELO", yVelo);

        m_drivetrain.setControl(
            m_request
                .withVelocityX(xVelo)
                .withVelocityY(yVelo)
                .withTargetDirection(m_targetRotation)
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
        m_logger.info("Stopping, because I finished");
    }

    @Override
    public boolean isFinished() {
        double errX = m_drivetrain.getPose().getX() - m_goal.getX();
        double errY = m_drivetrain.getPose().getY() - m_goal.getY();
        double err = Math.hypot(errX, errY);

        double errRotation = Math.abs(m_drivetrain.getPose()
            .getRotation()
            .minus(m_targetRotation)
            .getRadians()
        );
        
        m_logger.info("err: {}, errRotation: {}", err, errRotation);

        return (err < AutonConstants.translationTolerance && errRotation < AutonConstants.rotationTolerance);
    }
}
