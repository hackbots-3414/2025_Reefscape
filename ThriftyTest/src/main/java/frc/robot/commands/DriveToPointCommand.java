package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPointCommand extends Command {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(DriveToPointCommand.class);

    private final PIDController xPIDController = new PIDController(DriveConstants.k_translationPID.kP, 0, 0);
    private final PIDController yPIDController = new PIDController(DriveConstants.k_translationPID.kP, 0, 0);

    private static double m_targetX = 0.0;
    private static double m_targetY = 0.0;

    private static Rotation2d m_targetRotation;

    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withHeadingPID(DriveConstants.k_rotationPID.kP, 0, 0)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final Pose2d goal;
    private final CommandSwerveDrivetrain m_drivetrain;

    public DriveToPointCommand(Pose2d pose, CommandSwerveDrivetrain drivetrain) {
        this.goal = pose;
        this.m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_targetX = goal.getX();
        m_targetY = goal.getY();

        m_targetRotation = goal.getRotation();
    }

    @Override
    public void execute() {
        Pose2d currPose = m_drivetrain.getPose();

        double xVelo = xPIDController.calculate(currPose.getX(), m_targetX);
        double yVelo = yPIDController.calculate(currPose.getY(), m_targetY);

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
    }

    @Override
    public boolean isFinished() {
        double errX = xPIDController.getError();
        double errY = yPIDController.getError();
        double err = Math.hypot(errX, errY);

        double errRotation = m_drivetrain.getRotation3d()
            .toRotation2d()
            .minus(m_targetRotation)
            .getRadians();

        return err < AutonConstants.translationTolerance && errRotation < AutonConstants.rotationTolerance;
    }
}
