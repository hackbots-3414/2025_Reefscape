package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotObserver;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopCommand extends Command {
    private final PIDController xPIDController = new PIDController(15, 0, 0);
    private final PIDController yPIDController = new PIDController(15, 0, 0);
    private final PIDController rotPIDController = new PIDController(Math.PI * 2, Math.PI*5, 0);

    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> rotSupplier;
    private final Supplier<Boolean> useOpenLoop;

    private final double MaxSpeed = DriveConstants.k_maxLinearSpeed;
    private final double MaxAngularRate = DriveConstants.k_maxAngularSpeed;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private boolean alreadyClosedLoop = false;

    private static double goalX = 0.0;
    private static double goalY = 0.0;
    private static double goalRot = 0.0;

    private double xVelo = 0.0;
    private double yVelo = 0.0;
    private double rotVelo = 0.0;

    private final double dt = 0.02;

    public TeleopCommand(CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
            Supplier<Double> rotSupplier, Supplier<Boolean> useOpenLoop) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.useOpenLoop = useOpenLoop;

        addRequirements(drivetrain);

        rotPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Pose2d currPose = drivetrain.getPose();
        goalX = currPose.getX();
        goalY = currPose.getY();
        goalRot = currPose.getRotation().getRadians();
        alreadyClosedLoop = false;
    }

    @Override
    public void execute() {
        if (useOpenLoop.get()) { // open loop code
            drivetrain.setControl(
                    drive.withVelocityX(-xSupplier.get() * MaxSpeed).withVelocityY(-ySupplier.get() * MaxSpeed)
                            .withRotationalRate(-rotSupplier.get() * MaxAngularRate));
            alreadyClosedLoop = false;
        } else { // closed loop code
            Pose2d currPose = drivetrain.getPose();
            if (!alreadyClosedLoop) {
                goalX = currPose.getX();
                goalY = currPose.getY();
                goalRot = currPose.getRotation().getRadians();
                alreadyClosedLoop = true;
            }

            goalX += (xSupplier.get() / Math.sqrt(2)) * MaxSpeed * dt;
            goalY += (ySupplier.get() / Math.sqrt(2)) * MaxSpeed * dt;
            goalRot += (rotSupplier.get() / Math.sqrt(2)) * MaxAngularRate * dt;

            if (goalRot > Math.PI || goalRot < -Math.PI) {
                goalRot = -goalRot;
            }

            Pose2d goal = new Pose2d(
                goalX,
                goalY,
                new Rotation2d(goalRot)
            );

            RobotObserver.getField().getObject("Drive Target").setPose(goal);

            xVelo = -xPIDController.calculate(currPose.getX(), goalX);
            yVelo = -yPIDController.calculate(currPose.getY(), goalY);
            rotVelo = rotPIDController.calculate(currPose.getRotation().getRadians(), goalRot);

            drivetrain.setControl(drive.withVelocityX(xVelo).withVelocityY(yVelo).withRotationalRate(rotVelo));
        }
    }

    public static void setGoalPose(Pose2d pose) {
        goalRot = pose.getRotation().getRadians();
        goalX = pose.getX();
        goalY = pose.getY();
    }
}
