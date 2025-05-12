package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.driveassist.APConstraints;
import frc.robot.driveassist.APProfile;
import frc.robot.driveassist.Autopilot;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  protected static final PIDConstants kTranslationPID = new PIDConstants(2, 0.0, 0.0);
  protected static final PIDConstants kRotationPID = new PIDConstants(1.5, 0.0, 0.0);

  public static class HeadingPID {
    protected static final double kP = 4.0;
  }

  private static final APConstraints kTightAutopilotConstraintsI = new APConstraints()
      .withAcceleration(5.5)
      .withDecceleration(1.3);

  private static final APConstraints kTightAutopilotConstraintsU = APConstraints.unlimited();

  private static final APProfile kTightProfile = new APProfile()
      .withConstraintsI(kTightAutopilotConstraintsI)
      .withConstraintsU(kTightAutopilotConstraintsU)
      .withErrorXY(Centimeters.of(2))
      .withErrorTheta(Degrees.of(2));

  public static final Autopilot kTightAutopilot = new Autopilot(kTightProfile);

  private static final APConstraints kLooseAutopilotConstraintsI =
      new APConstraints()
          .withAcceleration(8.5)
          .withDecceleration(4);

  private static final APConstraints kLooseAutopilotConstraintsU =
      new APConstraints()
          .withAcceleration(8.5)
          .withDecceleration(3);

  private static final APProfile kLooseProfile = new APProfile()
      .withConstraintsI(kLooseAutopilotConstraintsI)
      .withConstraintsU(kLooseAutopilotConstraintsU)
      .withErrorXY(Centimeters.of(10))
      .withErrorTheta(Degrees.of(10));

  protected static final Autopilot kLooseAutopilot = new Autopilot(kLooseProfile);

  protected static final PPHolonomicDriveController k_pathplannerHolonomicDriveController =
      new PPHolonomicDriveController(kTranslationPID, kRotationPID);

  protected static final double k_maxTeleopLinearSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  protected static final double k_maxTeleopAngularSpeed =
      RotationsPerSecond.of(1.5).in(RadiansPerSecond);

  protected static final LinearVelocity k_maxLinearSpeed = MetersPerSecond.of(4);
  protected static final LinearAcceleration k_maxLinearAcceleration =
      MetersPerSecondPerSecond.of(3);
  protected static final AngularVelocity k_maxAngularSpeed = RotationsPerSecond.of(2);
  protected static final AngularAcceleration k_maxAngularAcceleration =
      RotationsPerSecondPerSecond.of(2);

  protected static final double k_maxRotationalSpeed = k_maxLinearSpeed.in(MetersPerSecond)
      / (TunerConstants.kWheelRadius.in(Meters) * 2 * Math.PI);

  protected static final double k_closedLoopOverrideToleranceTranslation = 0.05;
  protected static final double k_closedLoopOverrideToleranceRotation = 0.05;
}

