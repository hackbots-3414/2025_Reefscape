package frc.robot.superstructure.states;

import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.driveassist.Autopilot;
import frc.robot.driveassist.APTarget;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class Align implements EnterableState {
  private final Autopilot m_autopilot;
  private final APTarget m_target;
  private boolean m_flip;

  /**
   * A state that controls the drivetrain and drives to a certain, given point on the field
   */
  public Align(Autopilot autopilot, APTarget target) {
    m_autopilot = autopilot;
    m_target = target;
  }

  /**
   * A state in which the robot is aligned with a given target
   *
   * This approach uses the tight Autopilot configuration from DriveConstants
   */
  public Align(APTarget target) {
    m_autopilot = DriveConstants.kTightAutopilot;
    m_target = target;
  }

  public Command build(Subsystems subsystems) {
    return new DeferredCommand(() -> subsystems.drivetrain().align(m_autopilot, target()),
        Set.of(subsystems.drivetrain()));
  }

  public Align allianceRelative() {
    m_flip = true;
    return this;
  }

  private APTarget target() {
    if (m_flip) {
      if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      }
    }
    return m_target;
  }
}
