package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopCommand extends Command {
    private final PIDController xPIDController = new PIDController(15, 0, 0);
    private final PIDController yPIDController = new PIDController(15, 0, 0);
    private final PIDController rotPIDController = new PIDController(Math.PI * 2, 0, 0);

    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> rotSupplier;
    private final Supplier<Boolean> useOpenLoop;

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private boolean alreadyClosedLoop = false;

    private double goalX = 0.0;
    private double goalY = 0.0;
    private double goalRot = 0.0;

    private double xVelo = 0.0;
    private double yVelo = 0.0;
    private double rotVelo = 0.0;

    private final double dt = 0.02;

    public TeleopCommand(CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rotSupplier, Supplier<Boolean> useOpenLoop) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.useOpenLoop = useOpenLoop;

        addRequirements(drivetrain);

        rotPIDController.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putData("X PID", xPIDController);
        SmartDashboard.putData("Y PID", yPIDController);
        SmartDashboard.putData("ROT PID", rotPIDController);
    }

    @Override
    public void execute() {
        if (useOpenLoop.get()) { // open loop code
            drivetrain.setControl(drive.withVelocityX(-xSupplier.get() * MaxSpeed).withVelocityY(-ySupplier.get() * MaxSpeed).withRotationalRate(-rotSupplier.get() * MaxAngularRate));
            alreadyClosedLoop = false;
        } else { // closed loop code
            Pose2d currPose = drivetrain.getPose();
            if (!alreadyClosedLoop) {
                goalX = currPose.getX();
                goalY = currPose.getY();
                goalRot = currPose.getRotation().getRadians();
                alreadyClosedLoop = true;
            }

            goalX += xSupplier.get() * MaxSpeed * dt;
            goalY += ySupplier.get() * MaxSpeed * dt;
            goalRot += rotSupplier.get() * MaxAngularRate * dt;

            if (goalRot > Math.PI || goalRot < -Math.PI) {
                goalRot = -goalRot;
            }

            xVelo = -xPIDController.calculate(currPose.getX(), goalX);
            yVelo = -yPIDController.calculate(currPose.getY(), goalY);
            rotVelo = rotPIDController.calculate(currPose.getRotation().getRadians(), goalRot);

            drivetrain.setControl(drive.withVelocityX(xVelo).withVelocityY(yVelo).withRotationalRate(rotVelo));
        }
    }
}
