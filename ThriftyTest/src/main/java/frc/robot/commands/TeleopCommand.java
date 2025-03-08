package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ButtonBindingConstants.DragonReins;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopCommand extends Command {
    // private final PIDController rotPIDController = new PIDController(Math.PI * 2, 0, 0);

    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> rotSupplier;
    private final BooleanSupplier useOpenLoop;

    private final double maxTranslationalVelocity = DriveConstants.k_maxTeleopLinearSpeed;
    private final double maxRotationalVelocity = DriveConstants.k_maxTeleopAngularSpeed;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxTranslationalVelocity * DragonReins.deadband).withRotationalDeadband(maxRotationalVelocity * DragonReins.deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric driveClosedLoop = new SwerveRequest.FieldCentric()
            .withDeadband(maxTranslationalVelocity * DragonReins.deadband).withRotationalDeadband(maxRotationalVelocity * DragonReins.deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    public TeleopCommand(CommandSwerveDrivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
            Supplier<Double> rotSupplier, BooleanSupplier useOpenLoop) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.useOpenLoop = useOpenLoop;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (useOpenLoop.getAsBoolean()) { // open loop code
            drivetrain.setControl(
                    drive.withVelocityX(-xSupplier.get() * maxTranslationalVelocity)
                        .withVelocityY(-ySupplier.get() * maxTranslationalVelocity)
                        .withRotationalRate(rotSupplier.get() * maxRotationalVelocity));
        } else { // closed loop code
            drivetrain.setControl(
                driveClosedLoop.withVelocityX(-xSupplier.get() * maxTranslationalVelocity)
                        .withVelocityY(-ySupplier.get() * maxTranslationalVelocity)
                        .withRotationalRate(rotSupplier.get() * maxRotationalVelocity));
        }
    }
}
