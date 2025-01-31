// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.ManualClimberCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.AutonomousUtil;
import frc.robot.vision.VisionHandler;

public class RobotContainer {
    private final Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final VisionHandler m_vision = new VisionHandler(drivetrain);

    public RobotContainer() {
        configureSubsystems();
        configureDriverBindings();
        configureOperatorBindings();
        configureButtonBoard(dragonReins);
        configureAutonChooser();
        m_vision.startThread();
    }

    // ********** BINDINGS **********

    private final CommandPS5Controller dragonReins = new CommandPS5Controller(0);
    private final CommandPS5Controller operator = new CommandPS5Controller(1);

    private double getX() {
        return -dragonReins.getRawAxis(1);
    }

    private double getY() {
        return dragonReins.getRawAxis(0);
    }

    private double getRot() {
        return -dragonReins.getRawAxis(2);
    }

    private boolean getUseOpenLoopButton() {
        return dragonReins.button(3).getAsBoolean();
    }

    private void configureDriverBindings() {
        drivetrain.setDefaultCommand(new TeleopCommand(drivetrain, this::getX, this::getY, this::getRot, this::getUseOpenLoopButton));

        dragonReins.button(1).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(telemetry::telemeterize);

        dragonReins.axisMagnitudeGreaterThan(0, 0.0)
                .or(() -> dragonReins.axisMagnitudeGreaterThan(1, 0.0).getAsBoolean())
                .onTrue(new InstantCommand(() -> AutonomousUtil.clearQueue())); // can queue paths whenever, so long as
                                                                                // no dragonReins input is there
    }

    private void configureSysId() {
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        dragonReins.button(1).and(dragonReins.button(3))
                .whileTrue(drivetrain.sysIdDynamicTranslation(Direction.kForward));
        dragonReins.button(1).and(dragonReins.button(4))
                .whileTrue(drivetrain.sysIdDynamicTranslation(Direction.kReverse));
        dragonReins.button(2).and(dragonReins.button(3))
                .whileTrue(drivetrain.sysIdDynamicTranslation(Direction.kForward));
        dragonReins.button(2).and(dragonReins.button(4))
                .whileTrue(drivetrain.sysIdDynamicTranslation(Direction.kReverse));
    }

    private void configureButtonBoard(CommandPS5Controller controller) {
        controller.button(1).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.A.value)));
        controller.button(2).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.B.value)));
        controller.button(3).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.C.value)));
        controller.button(4).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.D.value)));
        controller.button(5).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.E.value)));
        controller.button(6).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.F.value)));
        controller.button(7).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.G.value)));
        controller.button(8).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.H.value)));
        controller.button(9).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.I.value)));
        controller.button(10).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.J.value)));
        controller.button(11).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.K.value)));
        controller.button(12).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.L.value)));
        controller.button(13).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.FARHP.value)));
        controller.button(14).and(controller.button(15))
                .onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.CLOSEHP.value)));

        // code where u can only ever queue paths while button is held, and when let go, queue will clear
        controller.button(15).onFalse(new InstantCommand(() -> AutonomousUtil.clearQueue()));
    }

    private void configureOperatorBindings() {
        operator.button(1).onTrue(scoreCommand(1));
        operator.button(2).onTrue(scoreCommand(2));
        operator.button(3).onTrue(scoreCommand(3));
        operator.button(4).onTrue(scoreCommand(4));
        // operator.button(1).whileTrue(new ManualPivot(pivot, true));
        // operator.button(2).whileTrue(new ManualPivot(pivot, false));
        // operator.button(3).whileTrue(new ManualElevator(elevator, true));
        // operator.button(4).whileTrue(new ManualElevator(elevator, false));
        operator.cross().whileTrue(new ManualClimberCommand(climber));
    }

    public enum ScoringLocations {
        A(new Pose2d(3.16, 4.19, Rotation2d.fromDegrees(0))),
        B(new Pose2d(3.16, 3.875, Rotation2d.fromDegrees(0))),
        C(new Pose2d(3.675, 3, Rotation2d.fromDegrees(60))),
        D(new Pose2d(4, 2.78, Rotation2d.fromDegrees(60))),
        E(new Pose2d(5, 2.8, Rotation2d.fromDegrees(120))),
        F(new Pose2d(5.3, 3, Rotation2d.fromDegrees(120))),
        G(new Pose2d(5.8, 3.85, Rotation2d.fromDegrees(180))),
        H(new Pose2d(5.8, 4.2, Rotation2d.fromDegrees(180))),
        I(new Pose2d(5.3, 5.1, Rotation2d.fromDegrees(-120))),
        J(new Pose2d(5, 5.25, Rotation2d.fromDegrees(-120))),
        K(new Pose2d(4, 5.25, Rotation2d.fromDegrees(-60))),
        L(new Pose2d(3.675, 5.1, Rotation2d.fromDegrees(-60))),
        FARHP(new Pose2d(1.194, 1.026, Rotation2d.fromDegrees(55))),
        CLOSEHP(new Pose2d(1.217, 7.012, Rotation2d.fromDegrees(-55)));

        private Pose2d value;

        private ScoringLocations(Pose2d value) {
            this.value = value;
        }
    }

    // ********** AUTONOMOUS **********

    // private final SendableChooser<Command> autoChooser;

    private final ArrayList<SendableChooser<Pose2d>> scoringLocationsChooser = new ArrayList<>();
    private final SendableChooser<Pose2d> pickupLocation = new SendableChooser<>();
    private final ArrayList<SendableChooser<Supplier<Command>>> scoringHeightsChooser = new ArrayList<>();

    private void configureAutonChooser() {
        // boolean isCompetition = true;

        // Filter any autos with the "comp" prefix
        // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        // (stream) -> isCompetition
        // ? stream.filter(auto -> auto.getName().startsWith("comp"))
        // : stream
        // );

        // autoChooser = AutoBuilder.buildAutoChooser();

        // SmartDashboard.putData("Auto Chooser", autoChooser);

        for (int i = 0; i < AutonConstants.numWaypoints; i++) {
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
        chooser.addOption("L1", () -> scoreCommand(1));
        chooser.addOption("L2", () -> scoreCommand(2));
        chooser.addOption("L3", () -> scoreCommand(3));
        chooser.setDefaultOption("L4", () -> scoreCommand(4));
    }

    public Command getAutonomousCommand() {
        Pose2d[] locations = new Pose2d[scoringLocationsChooser.size()];
        for (int i = 0; i < scoringLocationsChooser.size(); i++) {
            locations[i] = scoringLocationsChooser.get(i).getSelected();
        }

        Command[] heights = new Command[scoringHeightsChooser.size()];
        for (int i = 0; i < scoringHeightsChooser.size(); i++) {
            heights[i] = scoringHeightsChooser.get(i).getSelected().get();
        }

        return AutonomousUtil.generateRoutineWithCommands(pickupLocation.getSelected(), locations, heights, () -> scoreCommand(1));
    }

    // ********** SUBSYSTEMS **********

    private Elevator elevator;
    private Pivot pivot;
    private Climber climber;

    private void configureSubsystems() {
        elevator = new Elevator();
        pivot = new Pivot();
        climber = new Climber();
    }

    private Command scoreCommand(int level) {
        return new ScoreCommand(level, elevator);
    }
}
