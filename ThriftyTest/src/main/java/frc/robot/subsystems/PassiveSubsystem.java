package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents a subsystem with the ability to enable and disable its passive state.
 *
 * Because some subsystems have default, or "passive", behavior that should only run upon a certain
 * condition. The PassiveSubsystem deals with this functionality by storing a "taken" flag that is
 * set and unset by commands, methods, etc.
 *
 * Instead of using a default command to add passive behavior, this makes more sense because this
 * ensures that the
 */
public abstract class PassiveSubsystem extends SubsystemBase {
  private boolean m_taken;

  protected PassiveSubsystem() {
    enablePassiveBehavior();
  }

  /**
   * Returns whether the subsystem is taken, or available for passive action
   */
  public boolean taken() {
    return m_taken;
  }

  /**
   * Sets the subsystem's taken state to true so that passive behavior does not run.
   */
  public void take() {
    m_taken = true;
  }

  /**
   * Sets the subsystem's taken state to false so that passive behavior may run.
   */
  public void release() {
    m_taken = false;
  }

  /**
   * Calls <code>release()</code> if the condition is met. This is useful as a helper method to only
   * release a subsystem if a command was interrupted.
   */
  public void conditionalRelease(boolean shouldRelease) {
    if (shouldRelease) {
      release();
    }
  }

  /**
   * The passive method that should be called every sceduler iteration if and only if the subsystem
   * is not taken.
   */
  protected abstract void passive();

  private Command passiveBehavior() {
    return Commands.run(() -> {
      if (!taken()) {
        passive();
      }
    }, this);
  }

  /**
   * Enables passive behavior for the PassiveSubsystem. Without calling this method, passive
   * behavior will never be triggered. A good place for this is most likely the constructor for the
   * subsystem.
   */
  protected void enablePassiveBehavior() {
    setDefaultCommand(passiveBehavior());
  }
}
