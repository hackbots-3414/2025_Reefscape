package frc.robot.binding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.driveassist.APTarget;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.Align;
import frc.robot.superstructure.states.LowerReefAlgaeIntake;
import frc.robot.superstructure.states.Test;

public class DashboardBindings implements Binder {
  public void bind(Superstructure superstructure) {
    SmartDashboard.putData("Test/LowReefAlgaeIntake",
        superstructure.enter(new LowerReefAlgaeIntake()));
    SmartDashboard.putData("Test/Enter test state", superstructure.enter(new Test()));
    SmartDashboard.putData("Test/Drive To Center", superstructure.enter(new Align(
        new APTarget(new Pose2d(8, 4, Rotation2d.kZero))
            .withEntryAngle(Rotation2d.kZero))));
    SmartDashboard.putData("Test vision", superstructure.enter(new Align(
            new APTarget(new Pose2d(8.785400, 6.990802, Rotation2d.fromDegrees(-21.621))))));
  }
}
