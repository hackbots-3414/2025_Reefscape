package frc.robot.binding;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralLevel;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.CoralScore;
import frc.robot.superstructure.states.SeedPose;

public class DashboardBindings implements Binder {
  public void bind(Superstructure superstructure) {
    SmartDashboard.putData("Prep/Set Center", superstructure.enter(SeedPose.center()));
    SmartDashboard.putData("Prep/Set Left", superstructure.enter(SeedPose.left()));
    SmartDashboard.putData("Prep/Set Right", superstructure.enter(SeedPose.right()));
    SmartDashboard.putData("Test/L4", superstructure.enter(new CoralScore(CoralLevel.L4)));
  }
}
