package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathPlannerOverride extends Command {
    private final PIDController xPIDController = new PIDController(15, 0, 0);
    private final PIDController yPIDController = new PIDController(15, 0, 0);
    private final PIDController rotPIDController = new PIDController(Math.PI * 2, Math.PI*5, 0);

    private final double MaxSpeed = DriveConstants.k_maxLinearSpeed;
    private final double MaxAngularRate = DriveConstants.k_maxAngularSpeed;

    private static double goalX = 0.0;
    private static double goalY = 0.0;
    private static double goalRot = 0.0;

    private final SwerveRequest.FieldCentric driveClosedLoop = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private final Pose2d goal;
    private final CommandSwerveDrivetrain drivetrain;

    public PathPlannerOverride(Pose2d pose, CommandSwerveDrivetrain drivetrain) {
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

        if (goalRot > Math.PI || goalRot < -Math.PI) {
            goalRot = -goalRot;
        }
    }

    @Override
    public void execute() {
        Pose2d currPose = drivetrain.getBluePose();

        double xVelo = xPIDController.calculate(currPose.getX(), goalX);
        double yVelo = yPIDController.calculate(currPose.getY(), goalY);
        double rotVelo = rotPIDController.calculate(currPose.getRotation().getRadians(), goalRot);

        drivetrain.setControl(driveClosedLoop.withVelocityX(xVelo).withVelocityY(yVelo).withRotationalRate(rotVelo));
    }

    @Override
    public boolean isFinished() {
        return  (Math.abs(xPIDController.getError()) < AutonConstants.overrideTolerance)
            &&  (Math.abs(yPIDController.getError()) < AutonConstants.overrideTolerance)
            &&  (Math.abs(rotPIDController.getError()) < AutonConstants.degreeTolerance);
    }
}
