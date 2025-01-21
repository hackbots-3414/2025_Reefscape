package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopCommand extends Command {
    private PIDController xPIDController = new PIDController(1, 0, 0.1);
    private PIDController yPIDController = new PIDController(1, 0, 0.1);
    private PIDController rotPIDController = new PIDController(Math.PI / 4, 0, 0.1);

    private CommandSwerveDrivetrain drivetrain;
    private Supplier<Double> xSupplier;
    private Supplier<Double> ySupplier;
    private Supplier<Double> rotSupplier;
    private Supplier<Boolean> useOpenLoop;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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

    private double dt = 0.02;

    public TeleopCommand(CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> rotSupplier, Supplier<Boolean> useOpenLoop) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.useOpenLoop = useOpenLoop;

        xPIDController.setTolerance(0.5);
        yPIDController.setTolerance(0.5);
        rotPIDController.setTolerance(0.5);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!useOpenLoop.get()) { // open loop code
            drivetrain.setControl(drive.withVelocityX(xSupplier.get() * MaxSpeed).withVelocityY(ySupplier.get() * MaxSpeed).withRotationalRate(rotSupplier.get() * MaxAngularRate));
            alreadyClosedLoop = false;
        } else { // closed loop code
            Pose2d currPose = drivetrain.getPose();
            if (!alreadyClosedLoop) {
                goalX = currPose.getX();
                goalY = currPose.getY();
                goalRot = currPose.getRotation().getRadians();
                alreadyClosedLoop = true;
            }

            goalX += xSupplier.get().doubleValue() > 0.1 || xSupplier.get().doubleValue() < -0.1 ? xSupplier.get().doubleValue() * MaxSpeed * dt : 0; // add trig here to prevent (1,1)
            goalY += ySupplier.get().doubleValue() > 0.1 || ySupplier.get().doubleValue() < -0.1 ? ySupplier.get().doubleValue() * MaxSpeed * dt : 0; // add trig here to prevent (1,1)
            goalRot += rotSupplier.get().doubleValue() > 0.1 || rotSupplier.get().doubleValue() < -0.1 ? rotSupplier.get().doubleValue() * MaxAngularRate * dt : 0;

            xVelo = xPIDController.atSetpoint() ? 0 : xPIDController.calculate(currPose.getX(), goalX);
            yVelo = yPIDController.atSetpoint() ? 0 : yPIDController.calculate(currPose.getY(), goalY);
            rotVelo = rotPIDController.atSetpoint() ? 0 : rotPIDController.calculate(currPose.getRotation().getRadians(), goalRot);

            drivetrain.setControl(drive.withVelocityX(xVelo).withVelocityY(yVelo).withRotationalRate(rotVelo));
        }
    }
}
