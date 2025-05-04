package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.binding.Binder;
import frc.robot.binding.DriveBindings;
import frc.robot.binding.OperatorBindings;
import frc.robot.binding.RobotBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedFeedback;
import frc.robot.subsystems.Pivot;
import frc.robot.superstructure.Superstructure;
import frc.robot.vision.VisionHandler;

public class RobotContainer {
  private final PowerDistribution m_pdp = new PowerDistribution(1,ModuleType.kRev);

  private VisionHandler m_vision;

  private final Superstructure m_superstructure = new Superstructure(
      new AlgaeRollers(),
      new CoralRollers(),
      new Pivot(),
      new Elevator(),
      new Climber(),
      TunerConstants.createDrivetrain(),
      new LedFeedback());

  private final Binder m_driver = new DriveBindings();
  private final Binder m_operator = new OperatorBindings();
  private final Binder m_robot = new RobotBindings();

  public RobotContainer() {
    m_driver.bind(m_superstructure);
    m_operator.bind(m_superstructure);
    m_robot.bind(m_superstructure);
    m_vision = m_superstructure.buildVision();
    m_vision.startThread();

    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }

  public void enablePDPSwitch() {
    m_pdp.setSwitchableChannel(true);
  }
}
