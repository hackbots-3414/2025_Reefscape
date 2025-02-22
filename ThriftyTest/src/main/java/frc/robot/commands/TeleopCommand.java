package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopCommand extends Command {
    // private final PIDController rotPIDController = new PIDController(Math.PI * 2, 0, 0);

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

    private final SwerveRequest.FieldCentric driveClosedLoop = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private double xVelo = 0.0;
    private double yVelo = 0.0;
    private double rotVelo = 0.0;

    public TeleopCommand(CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
            Supplier<Double> rotSupplier, Supplier<Boolean> useOpenLoop) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.useOpenLoop = useOpenLoop;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // if (useOpenLoop.get()) { // open loop code
            drivetrain.setControl(
                    drive.withVelocityX(-xSupplier.get() * MaxSpeed)
                        .withVelocityY(-ySupplier.get() * MaxSpeed)
                        .withRotationalRate(-rotSupplier.get() * MaxAngularRate));
        // } else { // closed loop code
        //     xVelo = -xSupplier.get() * MaxSpeed;
        //     yVelo = -ySupplier.get() * MaxSpeed;
        //     rotVelo = rotSupplier.get() * MaxSpeed;

        //     drivetrain.setControl(driveClosedLoop.withVelocityX(xVelo).withVelocityY(yVelo).withRotationalRate(rotVelo));
        // }
    }
}
