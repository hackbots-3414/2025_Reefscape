package frc.robot.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotObserver;
import frc.robot.subsystems.LedFeedback;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.vision.tracking.AlgaeTracker;
import frc.robot.vision.localization.AprilTagVisionHandler;

public class Superstructure {
  private final Subsystems m_subsystems;

  /**
   * Constructs a new superstructure given the individual subsystems
   */
  public Superstructure(
      Algae algae,
      Coral coral,
      Pivot pivot,
      Elevator elevator,
      Climber climber,
      CommandSwerveDrivetrain drivetrain,
      LedFeedback leds) {

    m_subsystems = new Subsystems(
        algae,
        coral,
        pivot,
        elevator,
        climber,
        drivetrain,
        leds);

    drivetrain.setTippyTrigger(tippy());
    drivetrain.setSlowTrigger(elevator.unsafe());

    RobotObserver.setFFEnabledSupplier(elevator.unsafe().and(() -> !DriverStation.isAutonomous()));
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
   * Uses a <code>PassiveModifier</code> to change passive behavior
   */
  public void modify(PassiveModifier modifier, Trigger trigger) {
    modifier.modify(m_subsystems, trigger);
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

  public static record Subsystems(
      Algae algae,
      Coral coral,
      Pivot pivot,
      Elevator elevator,
      Climber climber,
      CommandSwerveDrivetrain drivetrain,
      LedFeedback leds) {
  }

  public Trigger aligned() {
    return m_subsystems.drivetrain().aligned();
  }

  public Trigger inReefZone() {
    return m_subsystems.drivetrain().inReefZone();
  }

  public Trigger holdingCoral() {
    return m_subsystems.coral().held();
  }

  public Trigger holdingAlgae() {
    return m_subsystems.algae.holdingAlgae();
  }

  public Trigger tippy() {
    return m_subsystems.elevator.unsafe().or(m_subsystems.algae.holdingAlgae());
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
