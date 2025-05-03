package frc.robot.binding;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonBindingConstants;
import frc.robot.Constants.ButtonBindingConstants.DragonReins;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.HeadingReset;
import frc.robot.superstructure.states.TeleopDrive;

public class DriveBindings implements Binder {
  private final CommandPS5Controller m_controller =
      new CommandPS5Controller(ButtonBindingConstants.driverPort);

  private final DoubleSupplier m_x;
  private final DoubleSupplier m_y;
  private final DoubleSupplier m_rot;

  private final Trigger m_resetHeading;
  private final Trigger m_processorAlign;

  public DriveBindings() {
    m_x = () -> m_controller.getRawAxis(DragonReins.xAxis) * (DragonReins.flipX ? -1.0 : 1.0);
    m_y = () -> m_controller.getRawAxis(DragonReins.yAxis) * (DragonReins.flipY ? -1.0 : 1.0);
    m_rot = () -> m_controller.getRawAxis(DragonReins.rotAxis) * (DragonReins.flipRot ? -1.0 : 1.0);

    m_resetHeading = m_controller.button(DragonReins.resetHeading);
    m_processorAlign = m_controller.button(DragonReins.processor);
  }

  public void bind(Superstructure superstructure) {
    superstructure.setDrive(superstructure.enter(new TeleopDrive(m_x, m_y, m_rot)));

    m_resetHeading.onTrue(superstructure.enter(new HeadingReset()));
    m_processorAlign.whileTrue(superstructure.enter(new Alignment());
  }
}
