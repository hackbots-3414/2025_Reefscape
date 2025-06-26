package frc.robot.superstructure.states;

import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;
import frc.robot.utils.FieldUtils;

public class Align implements EnterableState {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(Align.class);

  private Autopilot m_autopilot;
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
  
  public Align fast() {
    m_autopilot = DriveConstants.kFastAutopilot;
    return this;
  }

  private APTarget target() {
    if (m_flip) {
      return FieldUtils.flipTargetConditionally(m_target);
    }
    return m_target;
  }
}
