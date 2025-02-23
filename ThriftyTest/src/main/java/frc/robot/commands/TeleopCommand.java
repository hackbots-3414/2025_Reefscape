package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> rotSupplier;

    private final double MaxSpeed = DriveConstants.k_maxTeleopLinearSpeed;
    private final double MaxAngularRate = DriveConstants.k_maxTeleopAngularSpeed;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public TeleopCommand(CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
            Supplier<Double> rotSupplier, Supplier<Boolean> useOpenLoop) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setControl(
                drive.withVelocityX(-xSupplier.get() * MaxSpeed)
                .withVelocityY(-ySupplier.get() * MaxSpeed)
                .withRotationalRate(-rotSupplier.get() * MaxAngularRate));
    }
}
