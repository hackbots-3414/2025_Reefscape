package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.binding.Binder;
import frc.robot.binding.DashboardBindings;
import frc.robot.binding.DriveBindings;
import frc.robot.binding.NamedCommandBindings;
import frc.robot.binding.OperatorBindings;
import frc.robot.binding.RobotBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coral.CoralRollers;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.LedFeedback;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.algae.AlgaeRollers;
import frc.robot.superstructure.Superstructure;

public class RobotContainer {
  private final PowerDistribution m_pdp = new PowerDistribution(1, ModuleType.kRev);

  private SendableChooser<Command> m_autoChooser;

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
  private final Binder m_namedCommands = new NamedCommandBindings();
  private final Binder m_dashboard = new DashboardBindings();

  public RobotContainer() {
    m_driver.bind(m_superstructure);
    m_operator.bind(m_superstructure);
    m_robot.bind(m_superstructure);
    m_namedCommands.bind(m_superstructure);
    m_dashboard.bind(m_superstructure);

    m_superstructure.buildVision().startThread();

    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    m_autoChooser = AutoBuilder.buildAutoChooser();
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void enablePDPSwitch() {
    m_pdp.setSwitchableChannel(true);
  }
}
