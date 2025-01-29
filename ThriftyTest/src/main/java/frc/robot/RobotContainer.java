// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ScoringLocations;
import frc.robot.commands.Score;
import frc.robot.commands.TeleopCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AutonomousUtil;
import frc.robot.vision.VisionHandler;

public class RobotContainer {
    private final Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private final CommandPS5Controller dragonReins = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private VisionHandler m_vision = new VisionHandler(drivetrain);

    // private final SendableChooser<Command> autoChooser;

    private final ArrayList<SendableChooser<Pose2d>> scoringLocationsChooser = new ArrayList<>();
    private final SendableChooser<Pose2d> pickupLocation = new SendableChooser<>();
    private final ArrayList<SendableChooser<Supplier<Command>>> scoringHeightsChooser = new ArrayList<>();

    private final int numAutonWaypoints = 5;

    // private Elevator elevator;
    // private Coral coral;
    

    public RobotContainer() {
        configureBindings();
        m_vision.startThread();
        configureAutonChooser();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(new TeleopCommand(drivetrain, this::getX, this::getY, this::getRot, this::getUseOpenLoopButton));
        drivetrain.registerTelemetry(telemetry::telemeterize);
        dragonReins.axisMagnitudeGreaterThan(0, 0.0).or(() -> dragonReins.axisMagnitudeGreaterThan(1, 0.0).getAsBoolean()).onTrue(new InstantCommand(() -> AutonomousUtil.clearQueue())); // can queue paths whenever, so long as no dragonReins input is there
    }

    private double getX() {
        return -dragonReins.getLeftY();
    }

    private double getY() {
        return dragonReins.getLeftX();
    }

    private double getRot() {
        return -dragonReins.getRightX();
    }

    private boolean getUseOpenLoopButton() {
        return dragonReins.button(3).getAsBoolean();
    }

    private void bindButtonBoard(CommandXboxController controller) {
        controller.button(1).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.A.value)));
        controller.button(2).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.B.value)));
        controller.button(3).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.C.value)));
        controller.button(4).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.D.value)));
        controller.button(5).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.E.value)));
        controller.button(6).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.F.value)));
        controller.button(7).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.G.value)));
        controller.button(8).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.H.value)));
        controller.button(9).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.I.value)));
        controller.button(10).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.J.value)));
        controller.button(11).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.K.value)));
        controller.button(12).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.L.value)));
        controller.button(13).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.FARHP.value)));
        controller.button(14).and(controller.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.CLOSEHP.value)));

        controller.button(15).onFalse(new InstantCommand(() -> AutonomousUtil.clearQueue())); // code where u can only ever queue paths while button is held, and when let go, queue will clear
    }

    private void configureAutonChooser() {
        for (int i = 0; i < numAutonWaypoints; i++) {
            SendableChooser<Pose2d> placeChooser = new SendableChooser<>();
            configurePlaceChooser(placeChooser);
            scoringLocationsChooser.add(placeChooser);
            SmartDashboard.putData("Place to Score #" + (i), placeChooser);

            SendableChooser<Supplier<Command>> heightChooser = new SendableChooser<>();
            configureHeightChooser(heightChooser);
            scoringHeightsChooser.add(heightChooser);
            SmartDashboard.putData("Height to Score #" + (i), heightChooser);
        }        

        pickupLocation.setDefaultOption("CLOSE", ScoringLocations.CLOSEHP.value);
        pickupLocation.addOption("FAR", ScoringLocations.FARHP.value);
        SmartDashboard.putData(pickupLocation);
    }

    private void configurePlaceChooser(SendableChooser<Pose2d> chooser) {
        chooser.setDefaultOption("A", ScoringLocations.A.value);
        chooser.addOption("B", ScoringLocations.B.value);
        chooser.addOption("C", ScoringLocations.C.value);
        chooser.addOption("D", ScoringLocations.D.value);
        chooser.addOption("E", ScoringLocations.E.value);
        chooser.addOption("F", ScoringLocations.F.value);
        chooser.addOption("G", ScoringLocations.G.value);
        chooser.addOption("H", ScoringLocations.H.value);
        chooser.addOption("I", ScoringLocations.I.value);
        chooser.addOption("J", ScoringLocations.J.value);
        chooser.addOption("K", ScoringLocations.K.value);
        chooser.addOption("L", ScoringLocations.L.value);
    }

    private void configureHeightChooser(SendableChooser<Supplier<Command>> chooser) {
        chooser.addOption("L1", this::scoreL1);
        chooser.addOption("L2", this::scoreL2);
        chooser.addOption("L3", this::scoreL3);
        chooser.setDefaultOption("L4", this::scoreL4);
    }


    private Command scoreL1() {
        return makeScoreCommand(1);
    }

    private Command scoreL2() {
        return makeScoreCommand(2);
    }

    private Command scoreL3() {
        return makeScoreCommand(3);
    }

    private Command scoreL4() {
        return makeScoreCommand(4);
    }

    private Command makeScoreCommand(int level) {return new Score(level);}

    public Command getAutonomousCommand() {
        Pose2d[] locations = new Pose2d[scoringLocationsChooser.size()];
        for (int i = 0; i < scoringLocationsChooser.size(); i++) {
            locations[i] = scoringLocationsChooser.get(i).getSelected();
        }

        Command[] heights = new Command[scoringHeightsChooser.size()];
        for (int i = 0; i < scoringHeightsChooser.size(); i++) {
            heights[i] = scoringHeightsChooser.get(i).getSelected().get();
        }

        return AutonomousUtil.generateRoutineWithCommands(pickupLocation.getSelected(), locations, heights);
    }
}
