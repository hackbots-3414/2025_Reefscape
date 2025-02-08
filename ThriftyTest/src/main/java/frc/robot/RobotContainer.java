// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
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
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AlgaeEjectCommand;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeScoreCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralScoreCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRollers;
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
        configureButtonBoard(buttonBoard);
        configureAutonChooser();
        m_vision.startThread();
    }

    // ********** BINDINGS **********

    private final CommandPS5Controller dragonReins = new CommandPS5Controller(0);
    private final CommandPS5Controller operator = new CommandPS5Controller(1);
    private final CommandPS5Controller buttonBoard = new CommandPS5Controller(2);

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

        dragonReins.axisMagnitudeGreaterThan(0, DriveConstants.k_closedLoopOverrideTolerance)
                .or(() -> dragonReins.axisMagnitudeGreaterThan(1, DriveConstants.k_closedLoopOverrideTolerance).getAsBoolean())
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
        ScoringLocationsLeft[] locationsLeft = ScoringLocationsLeft.values();
        List<Pose2d> scoringLocationListLeft = new ArrayList<>();
        for (ScoringLocationsLeft location : locationsLeft) {
            scoringLocationListLeft.add(location.value);
        }

        ScoringLocationsRight[] locationsRight = ScoringLocationsRight.values();
        List<Pose2d> scoringLocationListRight = new ArrayList<>();
        for (ScoringLocationsRight location : locationsRight) {
            scoringLocationListRight.add(location.value);
        }
        controller.button(1).and(controller.button(11)).onTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(ButtonBoard.LEFT, () -> coralScoreCommand(1), scoringLocationListLeft, drivetrain)));
        controller.button(2).and(controller.button(11)).onTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(ButtonBoard.LEFT, () -> coralScoreCommand(2), scoringLocationListLeft, drivetrain)));
        controller.button(3).and(controller.button(11)).onTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(ButtonBoard.LEFT, () -> coralScoreCommand(3), scoringLocationListLeft, drivetrain)));
        controller.button(4).and(controller.button(11)).onTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(ButtonBoard.LEFT, () -> coralScoreCommand(4), scoringLocationListLeft, drivetrain)));
        controller.button(5).and(controller.button(11)).onTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(ButtonBoard.RIGHT, () -> coralScoreCommand(1), scoringLocationListRight, drivetrain)));
        controller.button(6).and(controller.button(11)).onTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(ButtonBoard.RIGHT, () -> coralScoreCommand(2), scoringLocationListRight, drivetrain)));
        controller.button(7).and(controller.button(11)).onTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(ButtonBoard.RIGHT, () -> coralScoreCommand(3), scoringLocationListRight, drivetrain)));
        controller.button(8).and(controller.button(11)).onTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(ButtonBoard.RIGHT, () -> coralScoreCommand(4), scoringLocationListRight, drivetrain)));

        controller.button(9).and(controller.button(11)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePathWithOverrides(ScoringLocations.FARHP.value, drivetrain, () -> coralIntakeCommand())));
        controller.button(10).and(controller.button(11)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePathWithOverrides(ScoringLocations.CLOSEHP.value, drivetrain, () -> coralIntakeCommand())));
        controller.button(11).onFalse(new InstantCommand(() -> AutonomousUtil.clearQueue()));
    }

    private void configureOperatorBindings() {
        operator.button(1).onTrue(coralScoreCommand(1));
        operator.button(2).onTrue(coralScoreCommand(2));
        operator.button(3).onTrue(coralScoreCommand(3));
        operator.button(4).onTrue(coralScoreCommand(4));
        operator.button(5).whileTrue(algaeIntakeCommand(AlgaeLocationPresets.REEFLOWER));
        operator.button(6).whileTrue(algaeIntakeCommand(AlgaeLocationPresets.REEFUPPER));
        operator.button(7).whileTrue(algaeIntakeCommand(AlgaeLocationPresets.GROUND));
        operator.button(8).whileTrue(algaeScoreCommand(AlgaeLocationPresets.NET));
        operator.button(9).whileTrue(algaeScoreCommand(AlgaeLocationPresets.PROCESSOR));
        operator.button(10).onTrue(new AlgaeEjectCommand(roller));
        // UNCOMMENT THE FOLLOWING FOR MANUAL MOVE FEATURES

        // operator.button(1).whileTrue(new ManualPivot(pivot, true));
        // operator.button(2).whileTrue(new ManualPivot(pivot, false));
        // operator.button(3).whileTrue(new ManualElevator(elevator, true));
        // operator.button(4).whileTrue(new ManualElevator(elevator, false));
        // operator.circle().whileTrue(new AlgaeRollerCommand(roller));
        // operator.cross().whileTrue(new ManualClimberCommand(climber));
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

    public enum ScoringLocationsLeft {
        A(ScoringLocations.A.value),
        C(ScoringLocations.C.value),
        E(ScoringLocations.E.value),
        G(ScoringLocations.G.value),
        I(ScoringLocations.I.value),
        K(ScoringLocations.K.value);

        private Pose2d value;

        private ScoringLocationsLeft(Pose2d value) {
            this.value = value;
        }
    }

    public enum ScoringLocationsRight {
        B(ScoringLocations.B.value),
        D(ScoringLocations.D.value),
        F(ScoringLocations.F.value),
        H(ScoringLocations.H.value),
        J(ScoringLocations.J.value),
        L(ScoringLocations.L.value);

        private Pose2d value;

        private ScoringLocationsRight(Pose2d value) {
            this.value = value;
        }
    }

    public enum ButtonBoard {
        LEFT, RIGHT;
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
        chooser.addOption("L1", () -> coralScoreCommand(1));
        chooser.addOption("L2", () -> coralScoreCommand(2));
        chooser.addOption("L3", () -> coralScoreCommand(3));
        chooser.setDefaultOption("L4", () -> coralScoreCommand(4));
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

        return AutonomousUtil.generateRoutineWithCommands(pickupLocation.getSelected(), locations, heights, this::stowElevatorCommand, this::coralIntakeCommand);
    }

    // ********** SUBSYSTEMS **********

    private Elevator elevator;
    private Pivot pivot;
    private Climber climber;
    private AlgaeRollers roller;
    private CoralRollers coral;

    private void configureSubsystems() {
        elevator = new Elevator();
        pivot = new Pivot();
        climber = new Climber();
        roller = new AlgaeRollers();
        coral = new CoralRollers();
    }

    private Command coralIntakeCommand() {
        return new CoralIntakeCommand(coral, elevator);
    }

    private Command coralScoreCommand(int level) {
        return new CoralScoreCommand(coral, elevator, level);
    }

    private Command algaeIntakeCommand(AlgaeLocationPresets intakeLocation) {
        return new AlgaeIntakeCommand(roller, elevator, pivot, drivetrain, intakeLocation);
    }

    private Command algaeScoreCommand(AlgaeLocationPresets scoreLocation) {
        return new AlgaeScoreCommand(roller, elevator, pivot, scoreLocation);
    }

    private Command stowElevatorCommand() {
        return new InstantCommand(() -> elevator.setStow());
    }

    public enum AlgaeLocationPresets {
        REEFLOWER, REEFUPPER, PROCESSOR, GROUND, NET;
    }
}
