package frc.robot.binding;

import static edu.wpi.first.units.Units.Meters;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ScoringLocations;
import frc.robot.subsystems.drivetrain.DriveConstants;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.AlgaeStow;
import frc.robot.superstructure.states.Align;
import frc.robot.superstructure.states.CoralScore;
import frc.robot.superstructure.states.CoralWait;
import frc.robot.superstructure.states.ElevatorZero;
import frc.robot.superstructure.states.HighGroundAlgaeIntake;
import frc.robot.superstructure.states.IntakeComplete;
import frc.robot.superstructure.states.LowerReefAlgaeIntake;
import frc.robot.superstructure.states.Net;
import frc.robot.superstructure.states.TrackAlgae;
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
    NamedCommands.registerCommand("Intake", superstructure.enter(new IntakeComplete()));
    NamedCommands.registerCommand("Recalibrate", superstructure.enter(new ElevatorZero()));

    /* algae */
    NamedCommands.registerCommand("Lower Algae", superstructure.enter(new LowerReefAlgaeIntake()));
    NamedCommands.registerCommand("Upper Algae", superstructure.enter(new UpperReefAlgaeIntake()));
    NamedCommands.registerCommand("Net", superstructure.enter(new Net()));
    NamedCommands.registerCommand("Algae Stow", superstructure.enter(new AlgaeStow()));
    NamedCommands.registerCommand("Lollipop", superstructure.enter(new HighGroundAlgaeIntake()));

    /* steal hehe */
    NamedCommands.registerCommand("Track", superstructure.enter(new TrackAlgae()));

    /* align */
    for (ScoringLocations location : Constants.ScoringLocations.values()) {
      String name = "Align ".concat(location.toString());
      NamedCommands.registerCommand(name, superstructure.enter(
          new Align(new APTarget(location.value).withEntryAngle(location.value.getRotation()))
              .allianceRelative()));
    }

    // HP stations
    APTarget lIntake = new APTarget(FieldConstants.kLeftIntake)
        .withRotationRadius(Meters.of(2.0));
    APTarget rIntake = new APTarget(FieldConstants.kRightIntake)
        .withRotationRadius(Meters.of(2.0));
    NamedCommands.registerCommand("Align LIntake", superstructure.enter(
        new Align(lIntake.withEntryAngle(Rotation2d.kPi))
            .allianceRelative()
            .fast()));
    NamedCommands.registerCommand("Align RIntake", superstructure.enter(
        new Align(rIntake.withEntryAngle(Rotation2d.kPi))
            .allianceRelative()
            .fast()));
    NamedCommands.registerCommand("Beeline LIntake", superstructure.enter(
        new Align(lIntake)
            .allianceRelative()
            .fast()));
    NamedCommands.registerCommand("Beeline RIntake", superstructure.enter(
        new Align(rIntake)
            .allianceRelative()
            .fast()));

    // Halfway points
    NamedCommands.registerCommand("Align IJ", superstructure.enter(
        new Align(new APTarget(FieldConstants.kIJ)
            .withEntryAngle(FieldConstants.kIJ.getRotation()))
                .allianceRelative()));
    NamedCommands.registerCommand("Align GH", superstructure.enter(
        new Align(new APTarget(FieldConstants.kGH)
            .withEntryAngle(FieldConstants.kGH.getRotation()))
                .allianceRelative()));
    // Barge
    APTarget bargeFromCenter = new APTarget(FieldConstants.kBargeFromCenter)
        .withEntryAngle(Rotation2d.fromDegrees(55.0))
        .withRotationRadius(Meters.of(1.5));
    APTarget bargeFromLeft = new APTarget(FieldConstants.kBargeFromLeft)
        .withEntryAngle(Rotation2d.fromDegrees(35.0))
        .withRotationRadius(Meters.of(1.5));
    NamedCommands.registerCommand("Align Barge Center", superstructure.enter(
        new Align(bargeFromCenter).allianceRelative())
        .onlyIf(superstructure.holdingAlgae()));
    NamedCommands.registerCommand("Align Barge Left", superstructure.enter(
        new Align(bargeFromLeft).allianceRelative())
        .onlyIf(superstructure.holdingAlgae()));
    NamedCommands.registerCommand("Beeline Barge Center", superstructure.enter(
        new Align(bargeFromCenter.withoutEntryAngle()).allianceRelative())
        .onlyIf(superstructure.holdingAlgae()));
    NamedCommands.registerCommand("Beeline Barge Left", superstructure.enter(
        new Align(bargeFromLeft.withoutEntryAngle()).allianceRelative())
        .onlyIf(superstructure.holdingAlgae()));

    // Steal!
    APTarget insideSteal = new APTarget(FieldConstants.kInsideSteal)
      .withRotationRadius(Meters.of(1));
    APTarget outsideSteal = new APTarget(FieldConstants.kOutsideSteal)
      .withRotationRadius(Meters.of(1));
    NamedCommands.registerCommand("Prepare Steal Inside",
        superstructure.enter(new Align(insideSteal).allianceRelative()));
    NamedCommands.registerCommand("Prepare Steal Outside",
        superstructure.enter(new Align(outsideSteal).allianceRelative()));
  }

}
