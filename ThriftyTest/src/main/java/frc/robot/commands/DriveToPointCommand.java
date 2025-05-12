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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotObserver;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldUtils;

public class DriveToPointCommand extends Command {
    private final Logger m_logger = LoggerFactory.getLogger(DriveToPointCommand.class);

    private final double dt = 0.02;

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
        m_drivetrain.setAligned(false);
        // flip goal if necessary
        if (m_flip) {
            m_goal = FieldUtils.flipPose(m_goal);
            m_flip = false;
        }
        m_targetRotation = m_goal.getRotation();

        RobotObserver.getField().getObject("target").setPose(m_goal);
    }

    @Override
    public void execute() {
        Translation2d adjusted = adjust(m_drivetrain.getPose(), m_goal);
        if (adjusted.getX() == 0 || adjusted.getY() == 0){
            m_logger.info("X/Y is Zero, current pose is {}, velocity is {}", m_drivetrain.getPose(), m_drivetrain.getVelocityComponents());
        }
        m_drivetrain.setControl(m_request
            .withVelocityX(adjusted.getX())
            .withVelocityY(adjusted.getY())
            .withTargetDirection(m_targetRotation)
        );
    }

    private Translation2d adjust(Pose2d current, Pose2d goal) {
        Translation2d robotToTarget = goal.getTranslation().minus(current.getTranslation());
        
        double distance = robotToTarget.getNorm();

        if (distance == 0) {
            return Translation2d.kZero;
        }

        double theoreticalMaxVelocity = Math.sqrt(2 * distance * DriveConstants.kMaxAccelerationTowardsTarget);
        // SmartDashboard.putNumber("Theoretical Max Velocity", theoreticalMaxVelocity);

        Translation2d currentVelocity = m_drivetrain.getVelocityComponents();
        // SmartDashboard.putNumber("Current Velocity", currentVelocity.getNorm());

        Translation2d direction = robotToTarget.div(distance);

        if (currentVelocity.getNorm() == 0) {
            m_logger.trace("Current Velocity was Zero");
            double velocity = Math.min(theoreticalMaxVelocity, dt * DriveConstants.kMaxAccelerationTowardsTarget);

            return direction.times(velocity);
        }

        double dot = direction.getX() * currentVelocity.getX() + direction.getY() * currentVelocity.getY();

        if (dot == 0) {
            m_logger.trace("Dot was Zero, currentVelocity {}, Distance {}, Direction {}", currentVelocity, distance, direction);
            // if we are completely perpendicular with the ideal translation, we can assume that current velocity IS u
            double velocityI = Math.min(theoreticalMaxVelocity, dt * DriveConstants.kMaxAccelerationTowardsTarget);
            Translation2d veloI = direction.times(velocityI);

            double adjustmentU = Math.min(dt * DriveConstants.kMaxAccelerationPerpendicularToTarget, currentVelocity.getNorm());
            // currentVelocity - maxAdjustmentU * currentVelocity hat
            Translation2d directionU = currentVelocity.div(currentVelocity.getNorm());
            Translation2d veloU = currentVelocity.minus(directionU.times(adjustmentU));

            return veloI.plus(veloU);
        }

        double currentVelocityTowardsTarget = Math.pow(currentVelocity.getNorm(), 2) / dot;
        // SmartDashboard.putNumber("Current V towards target", currentVelocityTowardsTarget);
        Translation2d currentVeloI = direction.times(currentVelocityTowardsTarget);

        Translation2d currentVeloU = currentVelocity.minus(currentVeloI);
        double currentVelocityPerpendicularToTarget = currentVeloU.getNorm();

        double adjustmentI = Math.min(theoreticalMaxVelocity, dt * DriveConstants.kMaxAccelerationTowardsTarget + currentVelocityTowardsTarget);
        Translation2d veloI = direction.times(adjustmentI);

        double adjustmentU = Math.min(currentVeloU.getNorm(), dt * DriveConstants.kMaxAccelerationPerpendicularToTarget);
        Translation2d veloU = Translation2d.kZero;
        if (currentVelocityPerpendicularToTarget != 0) {
            Translation2d directionU = currentVeloU.div(currentVelocityPerpendicularToTarget);
            veloU = currentVeloU.minus(directionU.times(adjustmentU));
        }

        // SmartDashboard.putNumber("Adjusted Velocity", veloI.getNorm());
        
        Translation2d r = veloI.plus(veloU);
        m_logger.trace("Adjust() returns {}" , r);

        return r;
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
