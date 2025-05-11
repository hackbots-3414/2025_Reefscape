package frc.robot.binding;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ScoringLocations;
import frc.robot.driveassist.APTarget;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.AlgaeStow;
import frc.robot.superstructure.states.Align;
import frc.robot.superstructure.states.CoralIntake;
import frc.robot.superstructure.states.CoralScore;
import frc.robot.superstructure.states.CoralWait;
import frc.robot.superstructure.states.LowerReefAlgaeIntake;
import frc.robot.superstructure.states.Net;
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
    NamedCommands.registerCommand("Lower Algae", superstructure.enter(new LowerReefAlgaeIntake()));
    NamedCommands.registerCommand("Upper Algae", superstructure.enter(new UpperReefAlgaeIntake()));
    NamedCommands.registerCommand("Net", superstructure.enter(new Net()));
    NamedCommands.registerCommand("Algae Stow", superstructure.enter(new AlgaeStow()));

    /* align */
    for (ScoringLocations location : Constants.ScoringLocations.values()) {
      String name = "Align ".concat(location.toString());
      NamedCommands.registerCommand(name, superstructure.enter(
          new Align(new APTarget(location.value).withEntryAngle(location.value.getRotation()))
              .allianceRelative()));
    }
    NamedCommands.registerCommand("Align LIntake", superstructure.enter(
        new Align(new APTarget(FieldConstants.kLeftIntake).withEntryAngle(Rotation2d.kPi))
            .allianceRelative()));
    NamedCommands.registerCommand("Align RIntake", superstructure.enter(
        new Align(new APTarget(FieldConstants.kRightIntake).withEntryAngle(Rotation2d.kPi))
            .allianceRelative()));
    NamedCommands.registerCommand("Align IJ", superstructure.enter(
        new Align(new APTarget(FieldConstants.kIJ).withEntryAngle(FieldConstants.kIJ.getRotation()))
            .allianceRelative()));
    NamedCommands.registerCommand("Align GH", superstructure.enter(
        new Align(new APTarget(FieldConstants.kGH).withEntryAngle(FieldConstants.kGH.getRotation()))
            .allianceRelative()));
    NamedCommands.registerCommand("Align Barge", superstructure.enter(
        new Align(new APTarget(FieldConstants.kBarge1)).allianceRelative()));
  }
}
