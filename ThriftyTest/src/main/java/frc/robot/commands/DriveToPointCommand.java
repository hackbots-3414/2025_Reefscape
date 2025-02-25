package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPointCommand extends Command {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(DriveToPointCommand.class);

    private final PIDController xPIDController = new PIDController(DriveConstants.k_translationPID.kP, 0, 0);
    private final PIDController yPIDController = new PIDController(DriveConstants.k_translationPID.kP, 0, 0);
    private final PIDController rotPIDController = new PIDController(DriveConstants.k_rotationPID.kP, 0, 0);

    private static double goalX = 0.0;
    private static double goalY = 0.0;
    private static double goalRot = 0.0;

    private final SwerveRequest.FieldCentric driveClosedLoop = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Pose2d goal;
    private final CommandSwerveDrivetrain drivetrain;

    public DriveToPointCommand(Pose2d pose, CommandSwerveDrivetrain drivetrain) {
        this.goal = pose;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        rotPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        goalX = goal.getX();
        goalY = goal.getY();
        goalRot = goal.getRotation().getRadians();

        //if (goalRot > Math.PI || goalRot < -Math.PI) {
        //    goalRot = -goalRot;
        //}
    }

    @Override
    public void execute() {
        Pose2d currPose = drivetrain.getPose();

        double xVelo = -xPIDController.calculate(currPose.getX(), goalX);
        double yVelo = -yPIDController.calculate(currPose.getY(), goalY);
        double rotVelo = rotPIDController.calculate(currPose.getRotation().getRadians(), goalRot);

        double xErr = currPose.getX() - goalX;
        double yErr = currPose.getY() - goalY;
        double err = Math.hypot(xErr, yErr);
        SmartDashboard.putNumber("closed looop error", err);

        drivetrain.setControl(driveClosedLoop.withVelocityX(xVelo).withVelocityY(yVelo).withRotationalRate(rotVelo));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        m_logger.warn("X ERROR: {}", Math.abs(xPIDController.getError()));
        m_logger.warn("Y ERROR: {}", Math.abs(yPIDController.getError()));
        m_logger.warn("ROT ERROR: {}", Math.abs(rotPIDController.getError()));

        return  (Math.abs(xPIDController.getError()) < AutonConstants.translationTolerance)
            &&  (Math.abs(yPIDController.getError()) < AutonConstants.translationTolerance)
            &&  (Math.abs(rotPIDController.getError()) < AutonConstants.rotationTolerance);
    }
}
