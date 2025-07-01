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
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  protected static final PIDConstants kTranslationPID = new PIDConstants(2, 0.0, 0.0);
  protected static final PIDConstants kRotationPID = new PIDConstants(1.5, 0.0, 0.0);

  public static class HeadingPID {
    protected static final double kP = 8.0;
  }

  private static final APConstraints kTightAutopilotPathConstraints = new APConstraints()
      .withAcceleration(15.0)
      .withJerk(1.5);

  private static final APProfile kTightProfile = new APProfile(kTightAutopilotPathConstraints)
      .withErrorXY(Centimeters.of(1))
      .withErrorTheta(Degrees.of(1))
      .withBeelineRadius(Centimeters.of(10));

  public static final Autopilot kTightAutopilot = new Autopilot(kTightProfile);

  private static final APConstraints kFastAutopilotPathConstraints =
      new APConstraints()
          .withAcceleration(20)
          .withJerk(8);

  private static final APProfile kFastProfile = new APProfile(kFastAutopilotPathConstraints)
      .withErrorXY(Centimeters.of(15))
      .withErrorTheta(Degrees.of(5))
      .withBeelineRadius(Centimeters.of(10));

  public static final Autopilot kFastAutopilot = new Autopilot(kFastProfile);

  protected static final PPHolonomicDriveController kPathplannerHolonomicDriveController =
      new PPHolonomicDriveController(kTranslationPID, kRotationPID);

  protected static final double kMaxTeleopLinearSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  protected static final double kMaxTeleopAngularSpeed =
      RotationsPerSecond.of(1.5).in(RadiansPerSecond);

  protected static final LinearVelocity kMaxLinearSpeed = MetersPerSecond.of(4);
  protected static final LinearAcceleration kMaxLinearAcceleration =
      MetersPerSecondPerSecond.of(3);
  protected static final AngularVelocity kMaxAngularSpeed = RotationsPerSecond.of(2);
  protected static final AngularAcceleration kMaxAngularAcceleration =
      RotationsPerSecondPerSecond.of(2);

  protected static final double kMaxRotationalSpeed = kMaxLinearSpeed.in(MetersPerSecond)
      / (TunerConstants.kWheelRadius.in(Meters) * 2 * Math.PI);

  protected static final double k_closedLoopOverrideToleranceTranslation = 0.05;
  protected static final double k_closedLoopOverrideToleranceRotation = 0.05;

  protected static final LinearVelocity kObjectTrackSpeed = MetersPerSecond.of(2);
  protected static final LinearVelocity kMaxObjectTrackingSpeed = MetersPerSecond.of(4);
}

