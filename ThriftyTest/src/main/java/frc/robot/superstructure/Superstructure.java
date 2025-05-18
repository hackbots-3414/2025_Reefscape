package frc.robot.superstructure;

import java.util.HashSet;
import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotObserver;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.LEDInputs;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.vision.VisionHandler;

public class Superstructure {
  private final Subsystems m_subsystems;

  private Set<Trigger> m_controllerConnections;

  /**
   * Constructs a new superstructure given the individual subsystems
   */
  public Superstructure(
      Algae algae,
      Coral coral,
      Pivot pivot,
      Elevator elevator,
      Climber climber,
      CommandSwerveDrivetrain drivetrain) {
    super();

    m_controllerConnections = new HashSet<>();
    LEDs leds = new LEDs(new LEDInputs(
          coral.holding(),
          coral.present(),
          allControllersOk(),
          climber.climbed()));
    m_subsystems = new Subsystems(
        algae,
        coral,
        pivot,
        elevator,
        climber,
        drivetrain,
        leds);

    RobotObserver.setFFEnabledSupplier(elevator.unsafe().and(() -> !DriverStation.isAutonomous()));
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
      Algae algae,
      Coral coral,
      Pivot pivot,
      Elevator elevator,
      Climber climber,
      CommandSwerveDrivetrain drivetrain,
      LEDs leds) {
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

  public Trigger allControllersOk() {
    return new Trigger(() -> {
      for (Trigger trigger : m_controllerConnections) {
        if (!trigger.getAsBoolean()) {
          return false;
        }
      }
      return true;
    });
  }

  public void addControllerCheck(Trigger trigger) {
    m_controllerConnections.add(trigger);
  }
}
