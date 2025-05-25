package frc.robot.binding;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.binding.BindingConstants.Driver;
import frc.robot.driveassist.APTarget;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.Align;
import frc.robot.superstructure.states.DeferredAlign;
import frc.robot.superstructure.states.DeferredAlign.AlignLocation;
import frc.robot.superstructure.states.HeadingReset;
import frc.robot.superstructure.states.TeleopDrive;

public class DriveBindings implements Binder {
  private final CommandPS5Controller m_controller =
      new CommandPS5Controller(BindingConstants.driverPort);

  private final DoubleSupplier m_x =
      () -> m_controller.getRawAxis(Driver.xAxis) * (Driver.flipX ? -1.0 : 1.0);;
  private final DoubleSupplier m_y =
      () -> m_controller.getRawAxis(Driver.yAxis) * (Driver.flipY ? -1.0 : 1.0);

  private final DoubleSupplier m_rot =
      () -> m_controller.getRawAxis(Driver.rotAxis) * (Driver.flipRot ? -1.0 : 1.0);

  private final Trigger m_resetHeading = m_controller.button(Driver.resetHeading);
  private final Trigger m_align = m_controller.button(Driver.processor);

  public void bind(Superstructure superstructure) {
    superstructure.setDrive(superstructure.enter(new TeleopDrive(m_x, m_y, m_rot)));

    m_resetHeading.onTrue(superstructure.enter(new HeadingReset()));
    m_align.and(superstructure.holdingAlgae()).whileTrue(superstructure.enter(new Align(
        new APTarget().withReference(FieldConstants.k_processor))));
    m_align.and(superstructure.holdingAlgae().negate()).whileTrue(superstructure.enter(
        new DeferredAlign(AlignLocation.Center)));
  }
}
