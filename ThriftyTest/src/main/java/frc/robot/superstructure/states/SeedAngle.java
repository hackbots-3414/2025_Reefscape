package frc.robot.superstructure.states;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class SeedAngle implements EnterableState {
    private final Rotation2d m_angle;

    public SeedAngle(Rotation2d angle) {
        m_angle = angle;
    }

    public static SeedAngle reverse() {
        return new SeedAngle(FieldConstants.kStartHeading);
    }

    public Command build(Subsystems subsystems) {
        return subsystems.drivetrain().setLocalHeading(m_angle);
    }
}
