package frc.robot.binding;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.Test;

public class DashboardBindings implements Binder {
  public void bind(Superstructure superstructure) {
    SmartDashboard.putData("Enter test state", superstructure.enter(new Test()));
  }
}
