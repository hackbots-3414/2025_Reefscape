package frc.robot.binding;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.CoralLevel;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.CoralIntake;
import frc.robot.superstructure.states.CoralScore;
import frc.robot.superstructure.states.CoralWait;
import frc.robot.superstructure.states.LowerReefAlgaeIntake;
import frc.robot.superstructure.states.UpperReefAlgaeIntake;

public class NamedCommandBindings implements Binder {
  /**
   * Configures PathPlanner's Named Commands
   */
  public NamedCommandBindings() {}

  public void bind(Superstructure superstructure) {
    /* coral */
    NamedCommands.registerCommand("L4", superstructure.enter(new CoralScore(CoralLevel.L4)));
    NamedCommands.registerCommand("L3", superstructure.enter(new CoralScore(CoralLevel.L3)));
    NamedCommands.registerCommand("Coral Wait", superstructure.enter(new CoralWait()));
    NamedCommands.registerCommand("Intake", superstructure.enter(new CoralIntake()));
    

    /* algae */
    NamedCommands.registerCommand("Algae Lower", superstructure.enter(new LowerReefAlgaeIntake()));
    NamedCommands.registerCommand("Algae Upper", superstructure.enter(new UpperReefAlgaeIntake()));
  }
}
