package frc.robot.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.vision.VisionHandler;

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

    RobotObserver.setFFEnabledSupplier(elevator.unsafe().and(() -> !DriverStation.isAutonomous()));
    SmartDashboard.putData("Coral/Subsystem", coral);
    SmartDashboard.putData("Elevator/Subsystem", elevator);
  }

  /**
   * Sets a specified <code>EnterableState</code> as reference state. This also sets that command to
   * run as a proxied command.
   */
  public Command enter(EnterableState state) {
    return enterWithoutProxy(state).asProxy();
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

  public void setDrive(Command driveCommand) {
    m_subsystems.drivetrain().setDefaultCommand(driveCommand);
  }

  public VisionHandler buildVision() {
    return new VisionHandler(
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
}
