// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ButtonBindingConstants;
import frc.robot.Constants.ClimbLocations;
import frc.robot.Constants.ButtonBindingConstants.BackupDriver;
import frc.robot.Constants.ButtonBindingConstants.ButtonBoard;
import frc.robot.Constants.ButtonBindingConstants.ButtonBoardAlternate;
import frc.robot.Constants.ButtonBindingConstants.ButtonBoardChoice;
import frc.robot.Constants.ButtonBindingConstants.DragonReins;
import frc.robot.Constants.ButtonBindingConstants.DriverChoice;
import frc.robot.Constants.CommandBounds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ReefClipLocations;
import frc.robot.Constants.ScoringLocations;
import frc.robot.Constants.ScoringLocationsLeft;
import frc.robot.Constants.ScoringLocationsMiddle;
import frc.robot.Constants.ScoringLocationsRight;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeScoreCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralScoreCommand;
import frc.robot.commands.ManualClimberCommand;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.ManualPivotCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.AutonomousUtil;
import frc.robot.utils.Shape;
import frc.robot.vision.VisionHandler;

public class RobotContainer {
    private final Telemetry m_telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    private final VisionHandler m_vision = new VisionHandler(m_drivetrain);

    public RobotContainer() {
        configureSubsystems();
        configureDriverBindings();
        configureButtonBoard();
        configureAutonChooser();
        m_vision.startThread();
        addBoundsToField();
    }

    private void addBoundsToField() {
        for (Map.Entry<String, Shape> entry : CommandBounds.displayBounds.entrySet()) {
            RobotObserver.getField().getObject(entry.getKey()).setPoses(
                entry.getValue().getVertices().stream()
                    .map(t -> new Pose2d(t.getX(), t.getY(), Rotation2d.kZero))
                    .collect(Collectors.toList())
            );
        }
    }

    // ********** BINDINGS **********

    private void configureDriverBindings() {
        CommandPS5Controller controller = new CommandPS5Controller(ButtonBindingConstants.driverPort);

        int xAxis;
        int yAxis;
        int rAxis; // rotation
        int resetHeading;
        int openLoop;

        double flipX;
        double flipY;
        double flipR;

        if (ButtonBindingConstants.driverChoice == DriverChoice.DRAGONREINS) {
            xAxis = DragonReins.xAxis;
            yAxis = DragonReins.yAxis;
            rAxis = DragonReins.rotAxis;

            resetHeading = DragonReins.resetHeading;
            openLoop = DragonReins.enableOpenLoop;

            flipX = DragonReins.flipX ? -1.0 : 1.0;
            flipY = DragonReins.flipY ? -1.0 : 1.0;
            flipR = DragonReins.flipRot ? -1.0 : 1.0;
        } else {
            xAxis = BackupDriver.xAxis;
            yAxis = BackupDriver.yAxis;
            rAxis = BackupDriver.rotAxis;

            resetHeading = BackupDriver.resetHeading;
            openLoop = BackupDriver.enableOpenLoop;

            flipX = BackupDriver.flipX ? -1.0 : 1.0;
            flipY = BackupDriver.flipY ? -1.0 : 1.0;
            flipR = BackupDriver.flipRot ? -1.0 : 1.0;
        }

        Supplier<Double> xSup = () -> controller.getRawAxis(xAxis) * flipX;
        Supplier<Double> ySup = () -> controller.getRawAxis(yAxis) * flipY;
        Supplier<Double> rSup = () -> controller.getRawAxis(rAxis) * flipR;
        Supplier<Boolean> openLoopSup = () -> controller.button(openLoop)
            .getAsBoolean();

        m_drivetrain.setDefaultCommand(
            new TeleopCommand(m_drivetrain, xSup, ySup, rSup, openLoopSup)
        );

        controller.button(resetHeading).onTrue(m_drivetrain.runOnce(() -> m_drivetrain.zeroPose()));

        controller.axisMagnitudeGreaterThan(xAxis, DriveConstants.k_closedLoopOverrideToleranceTranslation)
            .or(() -> controller.axisMagnitudeGreaterThan(yAxis, DriveConstants.k_closedLoopOverrideToleranceTranslation).getAsBoolean())
            .or(() -> controller.axisMagnitudeGreaterThan(rAxis, DriveConstants.k_closedLoopOverrideToleranceRotation).getAsBoolean())
            .onTrue(new InstantCommand(() -> AutonomousUtil.clearQueue()));
    }

    // private void configureSysId() {
    //     CommandPS5Controller controller = new CommandPS5Controller(3);

    //     controller.button(1).and(controller.button(3))
    //             .whileTrue(drivetrain.sysIdDynamicTranslation(Direction.kForward));
    //     controller.button(1).and(controller.button(4))
    //             .whileTrue(drivetrain.sysIdDynamicTranslation(Direction.kReverse));
    //     controller.button(2).and(controller.button(3))
    //             .whileTrue(drivetrain.sysIdDynamicTranslation(Direction.kForward));
    //     controller.button(2).and(controller.button(4))
    //             .whileTrue(drivetrain.sysIdDynamicTranslation(Direction.kReverse));
    // }

    private void configureButtonBoard() {
        // initializes lists of poses of all the enums - to use in auton util
        ScoringLocationsLeft[] locationsLeft = ScoringLocationsLeft.values();
        this.scoringLocationsListLeft = new ArrayList<>();
        for (ScoringLocationsLeft location : locationsLeft) {
            this.scoringLocationsListLeft.add(location.value);
        }

        ScoringLocationsRight[] locationsRight = ScoringLocationsRight.values();
        this.scoringLocationsRightList = new ArrayList<>();
        for (ScoringLocationsRight location : locationsRight) {
            this.scoringLocationsRightList.add(location.value);
        }

        ScoringLocationsMiddle[] locationsMiddle = ScoringLocationsMiddle.values();
        this.scoringLocationsMiddleList = new ArrayList<>();
        for (ScoringLocationsMiddle location : locationsMiddle) {
            this.scoringLocationsMiddleList.add(location.value);
        }

        ClimbLocations[] climbLocations = ClimbLocations.values();
        this.climbLocationsList = new ArrayList<>();
        for (ClimbLocations location : climbLocations) {
            this.climbLocationsList.add(location.value);
        }

        // handle bindings
        CommandPS5Controller controller = new CommandPS5Controller(ButtonBindingConstants.buttonBoardPort);

        if (ButtonBindingConstants.buttonBoardChoice == ButtonBoardChoice.BUTTONBOARD) {
            BooleanSupplier safetyOn = () -> controller.button(ButtonBoard.safetySwitch).getAsBoolean();
            BooleanSupplier safetyOff = () -> !controller.button(ButtonBoard.safetySwitch).getAsBoolean();

            // SAFETY **ON** MEANS USE THESE
            bindAutoCoralScoreCommand(1, ReefClipLocations.LEFT, controller.button(ButtonBoard.L1Auto).and(safetyOn));
            bindAutoCoralScoreCommand(2, ReefClipLocations.LEFT, controller.button(ButtonBoard.L2Auto).and(safetyOn));
            bindAutoCoralScoreCommand(3, ReefClipLocations.LEFT, controller.button(ButtonBoard.L3Auto).and(safetyOn));
            bindAutoCoralScoreCommand(4, ReefClipLocations.LEFT, controller.button(ButtonBoard.L4Auto).and(safetyOn));
            bindAutoCoralScoreCommand(1, ReefClipLocations.RIGHT, controller.button(ButtonBoard.R1Auto).and(safetyOn));
            bindAutoCoralScoreCommand(2, ReefClipLocations.RIGHT, controller.button(ButtonBoard.R2Auto).and(safetyOn));
            bindAutoCoralScoreCommand(3, ReefClipLocations.RIGHT, controller.button(ButtonBoard.R3Auto).and(safetyOn));
            bindAutoCoralScoreCommand(4, ReefClipLocations.RIGHT, controller.button(ButtonBoard.R4Auto).and(safetyOn));

            bindAutoCoralIntakeCommand(ReefClipLocations.LEFT, controller.button(ButtonBoard.leftIntake).and(safetyOn));
            bindAutoCoralIntakeCommand(ReefClipLocations.RIGHT, controller.button(ButtonBoard.rightIntake).and(safetyOn));

            bindAutoAlgaeCommand(AlgaeLocationPresets.GROUND, controller.button(ButtonBoard.groundAlgaeAuto).and(safetyOn));
            bindAutoAlgaeCommand(AlgaeLocationPresets.REEFLOWER, controller.button(ButtonBoard.lowAlgaeAuto).and(safetyOn));
            bindAutoAlgaeCommand(AlgaeLocationPresets.REEFUPPER, controller.button(ButtonBoard.highAlgaeAuto).and(safetyOn));
            bindAutoAlgaeCommand(AlgaeLocationPresets.NET, controller.button(ButtonBoard.netAuto).and(safetyOn));
            bindAutoAlgaeCommand(AlgaeLocationPresets.PROCESSOR, controller.button(ButtonBoard.processorAuto).and(safetyOn));

            bindManualClimbCommand(controller.button(ButtonBoard.climbAuto).and(safetyOn));

            // SAFETY **OFF** MEANS USE THESE
            bindManualCoralScoreCommand(1, controller.button(ButtonBoard.l1Score).and(safetyOff));
            bindManualCoralScoreCommand(2, controller.button(ButtonBoard.l2Score).and(safetyOff));
            bindManualCoralScoreCommand(3, controller.button(ButtonBoard.l3Score).and(safetyOff));
            bindManualCoralScoreCommand(4, controller.button(ButtonBoard.l4Score).and(safetyOff));

            bindManualCoralIntakeCommand(controller.button(ButtonBoard.intake).and(safetyOff));

            bindManualElevatorCommand(Direction.kForward, controller.button(ButtonBoard.manualElevatorUp).and(safetyOff));
            bindManualElevatorCommand(Direction.kReverse, controller.button(ButtonBoard.manualElevatorDown).and(safetyOff));

            bindManualPivotCommand(Direction.kForward, controller.button(ButtonBoard.manualPivotUp).and(safetyOff));
            bindManualPivotCommand(Direction.kReverse, controller.button(ButtonBoard.manualPivotDown).and(safetyOff));

            bindManualAlgaeCommand(AlgaeLocationPresets.GROUND, controller.button(ButtonBoard.groundAlgae).and(safetyOff));
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFLOWER, controller.button(ButtonBoard.lowAlgae).and(safetyOff));
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFUPPER, controller.button(ButtonBoard.highAlgae).and(safetyOff));
            bindManualAlgaeCommand(AlgaeLocationPresets.NET, controller.button(ButtonBoard.net).and(safetyOff));
            bindManualAlgaeCommand(AlgaeLocationPresets.PROCESSOR, controller.button(ButtonBoard.processor).and(safetyOff));

            bindManualClimbCommand(controller.button(ButtonBoard.climb).and(safetyOff));
        }

        if (ButtonBindingConstants.buttonBoardChoice == ButtonBoardChoice.BACKUP) {
            controller.button(ButtonBoardAlternate.safetySwitch).onChange(new InstantCommand(() -> RobotObserver.toggleSafety()));
            BooleanSupplier safety = () -> RobotObserver.getToggleSafety();

            // SAFETY **ON** MEANS USE THESE
            bindAutoCoralScoreCommand(1, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L1).and(() -> controller.button(ButtonBoardAlternate.leftReef).getAsBoolean()).and(safety));
            bindAutoCoralScoreCommand(2, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L2).and(() -> controller.button(ButtonBoardAlternate.leftReef).getAsBoolean()).and(safety));
            bindAutoCoralScoreCommand(3, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L3).and(() -> controller.button(ButtonBoardAlternate.leftReef).getAsBoolean()).and(safety));
            bindAutoCoralScoreCommand(4, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L4).and(() -> controller.button(ButtonBoardAlternate.leftReef).getAsBoolean()).and(safety));
            bindAutoCoralScoreCommand(1, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L1).and(() -> controller.button(ButtonBoardAlternate.rightReef).getAsBoolean()).and(safety));
            bindAutoCoralScoreCommand(2, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L2).and(() -> controller.button(ButtonBoardAlternate.rightReef).getAsBoolean()).and(safety));
            bindAutoCoralScoreCommand(3, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L3).and(() -> controller.button(ButtonBoardAlternate.rightReef).getAsBoolean()).and(safety));
            bindAutoCoralScoreCommand(4, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L4).and(() -> controller.button(ButtonBoardAlternate.rightReef).getAsBoolean()).and(safety));

            bindAutoCoralIntakeCommand(ReefClipLocations.LEFT, controller.button(ButtonBoardAlternate.leftIntake).and(safety));
            bindAutoCoralIntakeCommand(ReefClipLocations.RIGHT, controller.button(ButtonBoardAlternate.rightIntake).and(safety));

            bindAutoAlgaeCommand(AlgaeLocationPresets.GROUND, controller.button(ButtonBoardAlternate.groundAlgaeAuto).and(safety));
            bindAutoAlgaeCommand(AlgaeLocationPresets.REEFLOWER, controller.button(ButtonBoardAlternate.lowAlgaeAuto).and(safety));
            bindAutoAlgaeCommand(AlgaeLocationPresets.REEFUPPER, controller.button(ButtonBoardAlternate.highAlgaeAuto).and(safety));
            bindAutoAlgaeCommand(AlgaeLocationPresets.NET, controller.button(ButtonBoardAlternate.netAuto).and(safety));
            bindAutoAlgaeCommand(AlgaeLocationPresets.PROCESSOR, controller.button(ButtonBoardAlternate.processorAuto).and(safety));

            bindManualClimbCommand(controller.button(ButtonBoardAlternate.climbAuto).and(safety));

            // SAFETY **OFF** MEANS USE THESE
            bindManualCoralScoreCommand(1, controller.pov(ButtonBoardAlternate.l1Score).and(safety));
            bindManualCoralScoreCommand(2, controller.pov(ButtonBoardAlternate.l2Score).and(safety));
            bindManualCoralScoreCommand(3, controller.pov(ButtonBoardAlternate.l3Score).and(safety));
            bindManualCoralScoreCommand(4, controller.pov(ButtonBoardAlternate.l4Score).and(safety));

            bindManualCoralIntakeCommand(controller.button(ButtonBoardAlternate.intake).and(safety));

            bindManualElevatorCommand(Direction.kForward, controller.button(ButtonBoardAlternate.manualElevatorUp).and(safety));
            bindManualElevatorCommand(Direction.kReverse, controller.button(ButtonBoardAlternate.manualElevatorDown).and(safety));

            bindManualPivotCommand(Direction.kForward, controller.button(ButtonBoardAlternate.manualPivotUp).and(safety));
            bindManualPivotCommand(Direction.kReverse, controller.button(ButtonBoardAlternate.manualPivotDown).and(safety));

            bindManualAlgaeCommand(AlgaeLocationPresets.GROUND, controller.button(ButtonBoardAlternate.groundAlgae).and(safety));
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFLOWER, controller.button(ButtonBoardAlternate.lowAlgae).and(safety));
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFUPPER, controller.button(ButtonBoardAlternate.highAlgae).and(safety));
            bindManualAlgaeCommand(AlgaeLocationPresets.NET, controller.button(ButtonBoardAlternate.net).and(safety));
            bindManualAlgaeCommand(AlgaeLocationPresets.PROCESSOR, controller.button(ButtonBoardAlternate.processor).and(safety));

            bindManualClimbCommand(controller.button(ButtonBoardAlternate.climb).and(safety));
        }
    }

    public List<Pose2d> scoringLocationsListLeft;
    public List<Pose2d> scoringLocationsRightList;
    public List<Pose2d> scoringLocationsMiddleList;
    public List<Pose2d> climbLocationsList;

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

        return AutonomousUtil.generateRoutineWithCommands(pickupLocation.getSelected(), locations, heights, this::coralIntakeCommand);
    }

    // ********** SUBSYSTEMS **********

    private Elevator elevator;
    private Pivot pivot;
    private Climber climber;
    private AlgaeRollers roller;
    private CoralRollers coral;

    private void configureSubsystems() {
        m_drivetrain.registerTelemetry(m_telemetry::telemeterize);
        elevator = new Elevator();
        pivot = new Pivot();
        climber = new Climber();
        roller = new AlgaeRollers();
        coral = new CoralRollers();
        SmartDashboard.putData("Raise Elevator", new InstantCommand(() -> {
            elevator.setPosition(1.0);
        }));
        SmartDashboard.putData("Stow Elevator", new InstantCommand(() -> {
            elevator.setStow();
        }));
    }

    // ** BUTTON BOARD HELPERS **
    private void bindAutoCoralIntakeCommand(ReefClipLocations location, Trigger trigger) {
        switch (location) {
            case LEFT -> trigger.onTrue(new InstantCommand(() -> AutonomousUtil.queuePathWithCommand(m_drivetrain, ScoringLocations.FARHP.value, () -> coralIntakeCommand())));
            case RIGHT -> trigger.onTrue(new InstantCommand(() -> AutonomousUtil.queuePathWithCommand(m_drivetrain, ScoringLocations.CLOSEHP.value, () -> coralIntakeCommand())));
        }
    }

    private void bindManualCoralIntakeCommand(Trigger trigger) {
        trigger.whileTrue(coralIntakeCommand()); 
    }

    private void bindAutoCoralScoreCommand(int level, ReefClipLocations location, Trigger trigger) {
        switch (location) {
            case LEFT -> trigger.onTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(m_drivetrain, () -> coralScoreCommand(level), scoringLocationsListLeft)));
            case RIGHT -> trigger.onTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(m_drivetrain, () -> coralScoreCommand(level), scoringLocationsRightList)));
        }   
    }

    private void bindManualCoralScoreCommand(int level, Trigger trigger) {
        trigger.whileTrue(coralScoreCommand(level));    
    }

    private void bindAutoAlgaeCommand(AlgaeLocationPresets type, Trigger trigger) {
        switch (type) {
            case GROUND -> algaeIntakeCommand(AlgaeLocationPresets.GROUND);
            case REEFLOWER, REEFUPPER -> trigger.whileTrue(new InstantCommand(() -> AutonomousUtil.queuePathWithCommand(m_drivetrain, ScoringLocations.PROCESSOR.value, () -> algaeIntakeCommand(AlgaeLocationPresets.PROCESSOR))));
            case NET, PROCESSOR -> trigger.whileTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(m_drivetrain, () -> algaeScoreCommand(AlgaeLocationPresets.REEFUPPER), scoringLocationsMiddleList)));
        }
    }

    private void bindManualAlgaeCommand(AlgaeLocationPresets type, Trigger trigger) {
        switch (type) {
            case GROUND, REEFLOWER, REEFUPPER -> trigger.whileTrue(algaeIntakeCommand(type));
            case NET, PROCESSOR -> trigger.whileTrue(algaeScoreCommand(type));
        }
    }

    private void bindManualElevatorCommand(Direction direction, Trigger trigger) {
        switch (direction) {
            case kForward -> trigger.whileTrue(new ManualElevatorCommand(elevator, true));
            case kReverse -> trigger.whileTrue(new ManualElevatorCommand(elevator, false));
        }
    }

    private void bindManualPivotCommand(Direction direction, Trigger trigger) {
        switch (direction) {
            case kForward -> trigger.onTrue(new ManualPivotCommand(pivot, true));
            case kReverse -> trigger.onTrue(new ManualPivotCommand(pivot, false));
        }
    }

    private void bindManualClimbCommand(Trigger trigger) {
        trigger.whileTrue(new ManualClimberCommand(climber));
    }

    // ** SUBSYSTEM PASS IN HELPERS **

    private Command coralIntakeCommand() {
        return new CoralIntakeCommand(coral, elevator);
    }

    private Command coralScoreCommand(int level) {
        return new CoralScoreCommand(coral, elevator, level);
    }

    private Command algaeIntakeCommand(AlgaeLocationPresets intakeLocation) {
        return new AlgaeIntakeCommand(roller, elevator, pivot, intakeLocation);
    }

    private Command algaeScoreCommand(AlgaeLocationPresets scoreLocation) {
        return new AlgaeScoreCommand(roller, elevator, pivot, scoreLocation);
    }

    public enum AlgaeLocationPresets {
        REEFLOWER, REEFUPPER, PROCESSOR, GROUND, NET;
    }
}
