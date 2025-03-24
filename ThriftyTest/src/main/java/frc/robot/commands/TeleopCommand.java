package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ButtonBindingConstants.DragonReins;
import frc.robot.driveassist.DriverAssist;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Supplier<Double> m_xSupplier;
    private final Supplier<Double> m_ySupplier;
    private final Supplier<Double> m_rotSupplier;

    private final double maxTranslationalVelocity = DriveConstants.k_maxTeleopLinearSpeed;
    private final double maxRotationalVelocity = DriveConstants.k_maxTeleopAngularSpeed;

    private final DriverAssist m_assist = new DriverAssist();

    private final SwerveRequest.FieldCentric driveClosedLoop = new SwerveRequest.FieldCentric()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDeadband(maxTranslationalVelocity * DragonReins.deadband)
        .withRotationalDeadband(maxRotationalVelocity * DragonReins.deadband)
        .withDriveRequestType(DriveRequestType.Velocity);

    public TeleopCommand(
        CommandSwerveDrivetrain drivetrain,
        Supplier<Double> xSupplier,
        Supplier<Double> ySupplier,
        Supplier<Double> rotSupplier
    ) {
        m_drivetrain = drivetrain;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_rotSupplier = rotSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // calculate the field-relative speeds
        Transform2d robotRelative = new Transform2d(
            new Translation2d(
                m_xSupplier.get() * maxTranslationalVelocity,
                m_ySupplier.get() * maxTranslationalVelocity
            ),
            new Rotation2d(
                m_rotSupplier.get() * maxRotationalVelocity
            )
        );
        Transform2d fieldRelative = getFieldRelative(robotRelative);
        // avoid obstacles using drive assist
        Translation2d filtered = m_assist.calculate(
            fieldRelative.getTranslation(),
            m_drivetrain.getPose(),
            m_drivetrain.getNearestAntitarget()
        );
        Transform2d out = new Transform2d(filtered, fieldRelative.getRotation());
        applyVelocities(out);
    }

    /**
     * From a robot relative position, returns the field relative pose, using
     * the drivetrain's operator perspective
     */
    private Transform2d getFieldRelative(Transform2d robotRelative) {
        // get the offset
        Rotation2d forward = m_drivetrain.getOperatorForwardDirection();
        // get the original position
        double x = robotRelative.getX();
        double y = robotRelative.getY();
        Rotation2d theta = robotRelative.getRotation();
        // calculate the new position after rotation
        double px = x * forward.getCos() - y * forward.getSin();
        double py = y * forward.getCos() + x * forward.getSin();
        Rotation2d ptheta = theta;
        // combine the results
        return new Transform2d(new Translation2d(px, py), ptheta);
    }

    /**
     * Applies a transform2d with field relative velocities to the drivetrain
     */
    private void applyVelocities(Transform2d fieldRelative) {
        m_drivetrain.setControl(
            driveClosedLoop
                .withVelocityX(fieldRelative.getX())
                .withVelocityY(fieldRelative.getY())
                .withRotationalRate(fieldRelative.getRotation().getRadians())
        );
    }
}
