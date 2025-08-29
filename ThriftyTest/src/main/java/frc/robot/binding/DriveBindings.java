package frc.robot.binding;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.binding.BindingConstants.Driver;
import com.therekrab.autopilot.APTarget;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.Align;
import frc.robot.superstructure.states.DeferredAlign;
import frc.robot.superstructure.states.DeferredAlign.AlignLocation;
import frc.robot.superstructure.states.HeadingReset;
import frc.robot.superstructure.states.TeleopDrive;

public class DriveBindings implements Binder {
  private final CommandPS5Controller m_controller =
      new CommandPS5Controller(BindingConstants.kDriverPort);

  private final DoubleSupplier m_x =
      () -> m_controller.getRawAxis(Driver.xAxis) * (Driver.kFlipX ? -1.0 : 1.0);;
  private final DoubleSupplier m_y =
      () -> m_controller.getRawAxis(Driver.yAxis) * (Driver.kFlipY ? -1.0 : 1.0);

  private final DoubleSupplier m_rot =
      () -> m_controller.getRawAxis(Driver.rotAxis) * (Driver.kFlipRot ? -1.0 : 1.0);

  private final Trigger m_resetHeading = m_controller.button(Driver.kResetHeading);
  private final Trigger m_smartAlign = m_controller.button(Driver.kSmartAlign);
  private final Trigger m_leftAlign = m_controller.button(Driver.kLeftAlign);
  private final Trigger m_rightAlign = m_controller.button(Driver.kRightAlign);

  public void bind(Superstructure superstructure) {
    superstructure.setDrive(superstructure.enterWithoutProxy(new TeleopDrive(m_x, m_y, m_rot)));

    m_resetHeading.onTrue(superstructure.enter(new HeadingReset()));
    // m_smartAlign.and(superstructure.holdingAlgae()).whileTrue(superstructure.enter(new Align(
    //     new APTarget(FieldConstants.k_processor).withEntryAngle(Rotation2d.kCW_Pi_2))));
    m_smartAlign.and(superstructure.holdingAlgae().negate()).whileTrue(superstructure.enter(
        new DeferredAlign(AlignLocation.Center)));
    m_leftAlign.whileTrue(superstructure.enter(new DeferredAlign(AlignLocation.Left)));
    m_rightAlign.whileTrue(superstructure.enter(new DeferredAlign(AlignLocation.Right)));
  }
}
