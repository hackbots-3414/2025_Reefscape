package frc.robot.superstructure.states;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class TeleopDrive implements EnterableState {
  private final DoubleSupplier m_x;
  private final DoubleSupplier m_y;
  private final DoubleSupplier m_rot;

  /**
   * A state that is used to drive the robot given suppliers for x, y, and rotational velocities.
   */
  public TeleopDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    m_x = x;
    m_y = y;
    m_rot = rot;
  }

  public Command build(Subsystems subsystems) {
    return subsystems.drivetrain().teleopDrive(m_x, m_y, m_rot);
  }
}
