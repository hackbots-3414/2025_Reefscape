package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotObserver;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.vision.localization.AprilTagVisionHandler;
import frc.robot.vision.tracking.AlgaeTracker;

public class Superstructure {
  private final Subsystems m_subsystems;

  /**
   * Constructs a new superstructure given the individual subsystems
   */
  public Superstructure(CommandSwerveDrivetrain drivetrain) {

    m_subsystems = new Subsystems(drivetrain);

    drivetrain.setTippyTrigger(tippy());

    RobotObserver.setFFEnabledSupplier(() -> false);
  }

  /**
   * Sets a specified <code>EnterableState</code> as reference state. This also sets that command to
   * run as a proxied command.
   */
  public Command enter(EnterableState state) {
    return state.build(m_subsystems)
        .withName(state.getClass().getSimpleName())
        .asProxy();
  }

  /**
   * The same thing as <code>enter()</code>, except this is NOT a proxied command. This should be
   * used for default commands, where the command needs to explicity list its subsystems. However,
   * other than that, there aren't many uses for this method, so <b>use with care!</b>.
   */
  public Command enterWithoutProxy(EnterableState state) {
    return state.build(m_subsystems)
        .withName(state.getClass().getSimpleName()); // avoid poorly named commands
  }

  /**
   * Sets the provided command to be a default command for the drivetrain
   */
  public void setDrive(Command driveCommand) {
    m_subsystems.drivetrain().setDefaultCommand(driveCommand);
  }

  /**
   * Returns an <code>AprilTagVisionHandler</code>.
   */
  public AprilTagVisionHandler buildVision() {
    return new AprilTagVisionHandler(
        m_subsystems.drivetrain()::getPose,
        m_subsystems.drivetrain()::addPoseEstimate);
  }

  public static record Subsystems(CommandSwerveDrivetrain drivetrain) {
  }

  public Trigger aligned() {
    return m_subsystems.drivetrain().aligned();
  }

  public Trigger inReefZone() {
    return m_subsystems.drivetrain().inReefZone();
  }

  public Trigger holdingCoral() {
    return new Trigger(() -> false);
  }

  public Trigger holdingAlgae() {
    return new Trigger(() -> false);
  }

  public Trigger tippy() {
    return new Trigger(() -> false);
  }

  /**
   * Returns a runnable that can be used for tracking algae
   *
   * If algae tracking is disabled, then this Runnable does nothing.
   */
  public Runnable buildAlgaeTracker() {
    if (AlgaeTracker.enabled) {
      return new AlgaeTracker(
          m_subsystems.drivetrain()::getPose,
          m_subsystems.drivetrain()::addObjectTrackingData);
    } else {
      return () -> {
      };
    }
  }
}
