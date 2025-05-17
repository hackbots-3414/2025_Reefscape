package frc.robot.binding;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralLevel;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.CoralScore;

public class DashboardBindings implements Binder {
  public void bind(Superstructure superstructure) {
    SmartDashboard.putData("Test/L4", superstructure.enter(new CoralScore(CoralLevel.L4)));
  }
}
