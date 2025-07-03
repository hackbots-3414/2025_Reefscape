package frc.robot.binding;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.Subsystems;
import frc.robot.superstructure.states.SeedPose;
import frc.robot.superstructure.states.Stow;
import frc.robot.superstructure.states.TrackAlgae;

public class DashboardBindings implements Binder {
  public void bind(Superstructure superstructure) {
    SmartDashboard.putData("Prep/Set Center", superstructure.enter(SeedPose.center()));
    SmartDashboard.putData("Prep/Set Left", superstructure.enter(SeedPose.left()));
    SmartDashboard.putData("Prep/Set Right", superstructure.enter(SeedPose.right()));

    SmartDashboard.putData("Test/Stow", superstructure.enter(new Stow()));
    SmartDashboard.putData("Test/Elevator Up", superstructure.enter(new EnterableState() {
      public Command build(Subsystems subsystems) {
        return subsystems.elevator().go(ElevatorState.L4);
      }
    }));

    SmartDashboard.putData("Test/Follow Algae", superstructure.enter(new TrackAlgae()));
  }
}
