package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LedFeedback;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.algae.AlgaeRollers;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coral.CoralRollers;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.vision.VisionHandler;

public class Superstructure {
  private final Subsystems m_subsystems;

  /**
   * Constructs a new superstructure given the individual subsystems
   */
  public Superstructure(
      AlgaeRollers algaeRollers,
      CoralRollers coralRollers,
      Pivot pivot,
      Elevator elevator,
      Climber climber,
      CommandSwerveDrivetrain drivetrain,
      LedFeedback leds) {

    m_subsystems = new Subsystems(
        algaeRollers,
        coralRollers,
        pivot,
        elevator,
        climber,
        drivetrain,
        leds);
  }

  /**
   * Sets a specified <code>EnterableState</code> as reference state
   */
  public Command enter(EnterableState state) {
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
      AlgaeRollers algae,
      CoralRollers coral,
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
    return m_subsystems.coral().holding();
  }
}
