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

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.Constants.ButtonBindingConstants.BackupDriver;
import frc.robot.Constants.ButtonBindingConstants.ButtonBoard;
import frc.robot.Constants.ButtonBindingConstants.ButtonBoardAlternate;
import frc.robot.Constants.ButtonBindingConstants.ButtonBoardChoice;
import frc.robot.Constants.ButtonBindingConstants.DragonReins;
import frc.robot.Constants.ButtonBindingConstants.DriverChoice;
import frc.robot.Constants.ClimbLocations;
import frc.robot.Constants.CommandBounds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ReefClipLocations;
import frc.robot.Constants.ScoringLocations;
import frc.robot.Constants.ScoringLocationsLeft;
import frc.robot.Constants.ScoringLocationsMiddle;
import frc.robot.Constants.ScoringLocationsRight;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeScoreCommand;
import frc.robot.commands.CoralEjectCommand;
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
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(RobotContainer.class);
    @SuppressWarnings("unused")
    private final Telemetry m_telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    private final VisionHandler m_vision = new VisionHandler(m_drivetrain);

    public RobotContainer() {
        configureSubsystems();
        configureNamedCommands();
        configureDriverBindings();
        configureButtonBoard();
        configureAutonChooser();
        configureVision();
        // addBoundsToField();
        // configureSysId();
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

        controller.button(resetHeading).onTrue(m_drivetrain.runOnce(() -> m_drivetrain.resetHeading()));
        controller.button(resetHeading).onFalse(m_drivetrain.runOnce(() -> m_drivetrain.resetHeading()));

        controller.axisMagnitudeGreaterThan(xAxis, DriveConstants.k_closedLoopOverrideToleranceTranslation)
            .or(() -> controller.axisMagnitudeGreaterThan(yAxis, DriveConstants.k_closedLoopOverrideToleranceTranslation).getAsBoolean())
            .or(() -> controller.axisMagnitudeGreaterThan(rAxis, DriveConstants.k_closedLoopOverrideToleranceRotation).getAsBoolean())
            .onTrue(new InstantCommand(() -> AutonomousUtil.clearQueue()));
    }

    // private void configureSysId() {
    //     CommandPS5Controller controller = new CommandPS5Controller(3);

    //     controller.povUp().whileTrue(m_drivetrain.sysIdQuasistaticTranslation(Direction.kForward));
    //     controller.povDown().whileTrue(m_drivetrain.sysIdQuasistaticTranslation(Direction.kReverse));
    //     controller.povRight().whileTrue(m_drivetrain.sysIdDynamicTranslation(Direction.kForward));
    //     controller.povLeft().whileTrue(m_drivetrain.sysIdDynamicTranslation(Direction.kReverse));

    //     controller.triangle().whileTrue(m_drivetrain.sysIdQuasistaticRotation(Direction.kForward));
    //     controller.cross().whileTrue(m_drivetrain.sysIdQuasistaticRotation(Direction.kReverse));
    //     controller.circle().whileTrue(m_drivetrain.sysIdDynamicRotation(Direction.kForward));
    //     controller.square().whileTrue(m_drivetrain.sysIdDynamicRotation(Direction.kReverse));
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

        // m_coralRollers.setDefaultCommand(new CoralDefaultCommand(m_coralRollers));
        // m_elevator.setDefaultCommand(new InstantCommand(() -> m_elevator.setStow()));

        if (ButtonBindingConstants.buttonBoardChoice == ButtonBoardChoice.BUTTONBOARD) {
            controller.button(ButtonBoardAlternate.manualModeSwitch).onChange(new InstantCommand(() -> RobotObserver.toggleManualMode()));
            BooleanSupplier manualModeOn = () -> RobotObserver.getManualMode();
            BooleanSupplier manualModeOff = () -> !RobotObserver.getManualMode();


            // Manual Mode On
            bindManualCoralScoreCommand(1, controller.pov(ButtonBoard.L1).and(manualModeOn));
            bindManualCoralScoreCommand(2, controller.pov(ButtonBoard.L2).and(manualModeOn));
            bindManualCoralScoreCommand(3, controller.pov(ButtonBoard.L3).and(manualModeOn));
            bindManualCoralScoreCommand(4, controller.pov(ButtonBoard.L4).and(manualModeOn));

            bindManualCoralIntakeCommand(controller.button(ButtonBoard.intake).and(manualModeOn));

            // bindManualElevatorCommand(Direction.kForward, controller.axisGreaterThan(ButtonBoardAlternate.manualElevatorUp, ButtonBoardAlternate.manualThresholds).and(manualModeOn));
            // bindManualElevatorCommand(Direction.kReverse, controller.axisGreaterThan(ButtonBoardAlternate.manualElevatorDown, ButtonBoardAlternate.manualThresholds).and(manualModeOn));

            // bindManualPivotCommand(Direction.kForward, controller.axisGreaterThan(ButtonBoardAlternate.manualPivotUp, ButtonBoardAlternate.manualThresholds).and(manualModeOn));
            // bindManualPivotCommand(Direction.kReverse, controller.axisGreaterThan(ButtonBoardAlternate.manualPivotDown, ButtonBoardAlternate.manualThresholds).and(manualModeOn));

            bindManualAlgaeCommand(AlgaeLocationPresets.GROUND, controller.pov(ButtonBoard.groundAlgae).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFLOWER, controller.button(ButtonBoard.lowAlgae).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFUPPER, controller.button(ButtonBoard.highAlgae).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.NET, controller.pov(ButtonBoard.net).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.PROCESSOR, controller.pov(ButtonBoard.processor).and(manualModeOn));

            bindManualClimbCommand(controller.button(ButtonBoardAlternate.climb).and(manualModeOn));

        } else {
            controller.button(ButtonBoardAlternate.manualModeSwitch).onTrue(new InstantCommand(() -> RobotObserver.toggleManualMode()));
            BooleanSupplier manualModeOn = () -> RobotObserver.getManualMode();
            BooleanSupplier manualModeOff = () -> !RobotObserver.getManualMode();

            Trigger algaeOn = controller.button(ButtonBoardAlternate.algaeModeButton);

            controller.button(ButtonBoardAlternate.ejectCoral).whileTrue(new CoralEjectCommand(m_coralRollers, m_elevator));

            // Manual Mode Off
            bindAutoCoralScoreCommand(1, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L1).and(() -> controller.button(ButtonBoardAlternate.leftReef).getAsBoolean()).and(manualModeOff));
            bindAutoCoralScoreCommand(2, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L2).and(() -> controller.button(ButtonBoardAlternate.leftReef).getAsBoolean()).and(manualModeOff));
            bindAutoCoralScoreCommand(3, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L3).and(() -> controller.button(ButtonBoardAlternate.leftReef).getAsBoolean()).and(manualModeOff));
            bindAutoCoralScoreCommand(4, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L4).and(() -> controller.button(ButtonBoardAlternate.leftReef).getAsBoolean()).and(manualModeOff));
            bindAutoCoralScoreCommand(1, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L1).and(() -> controller.button(ButtonBoardAlternate.rightReef).getAsBoolean()).and(manualModeOff));
            bindAutoCoralScoreCommand(2, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L2).and(() -> controller.button(ButtonBoardAlternate.rightReef).getAsBoolean()).and(manualModeOff));
            bindAutoCoralScoreCommand(3, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L3).and(() -> controller.button(ButtonBoardAlternate.rightReef).getAsBoolean()).and(manualModeOff));
            bindAutoCoralScoreCommand(4, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L4).and(() -> controller.button(ButtonBoardAlternate.rightReef).getAsBoolean()).and(manualModeOff));

            bindAutoCoralIntakeCommand(ReefClipLocations.LEFT, controller.button(ButtonBoardAlternate.leftIntake).and(manualModeOff));
            bindAutoCoralIntakeCommand(ReefClipLocations.RIGHT, controller.button(ButtonBoardAlternate.rightIntake).and(manualModeOff));

            bindAutoAlgaeCommand(AlgaeLocationPresets.GROUND, controller.pov(ButtonBoardAlternate.groundAlgae).and(algaeOn).and(manualModeOff));
            bindAutoAlgaeCommand(AlgaeLocationPresets.REEFLOWER, controller.button(ButtonBoardAlternate.lowAlgae).and(manualModeOff));
            bindAutoAlgaeCommand(AlgaeLocationPresets.REEFUPPER, controller.button(ButtonBoardAlternate.highAlgae).and(manualModeOff));
            bindAutoAlgaeCommand(AlgaeLocationPresets.NET, controller.pov(ButtonBoardAlternate.net).and(algaeOn).and(manualModeOff));
            bindAutoAlgaeCommand(AlgaeLocationPresets.PROCESSOR, controller.pov(ButtonBoardAlternate.processor).and(algaeOn).and(manualModeOff));

            bindManualClimbCommand(controller.button(ButtonBoardAlternate.climb).and(manualModeOff));

            // Manual Mode On
            bindManualCoralScoreCommand(1, controller.pov(ButtonBoardAlternate.L1).and(manualModeOn));
            bindManualCoralScoreCommand(2, controller.pov(ButtonBoardAlternate.L2).and(manualModeOn));
            bindManualCoralScoreCommand(3, controller.pov(ButtonBoardAlternate.L3).and(manualModeOn));
            bindManualCoralScoreCommand(4, controller.pov(ButtonBoardAlternate.L4).and(manualModeOn));

            bindManualCoralIntakeCommand(controller.button(ButtonBoardAlternate.intake).and(manualModeOn));

            // bindManualElevatorCommand(Direction.kForward, controller.axisGreaterThan(ButtonBoardAlternate.manualElevatorUp, ButtonBoardAlternate.manualThresholds).and(manualModeOn));
            // bindManualElevatorCommand(Direction.kReverse, controller.axisGreaterThan(ButtonBoardAlternate.manualElevatorDown, ButtonBoardAlternate.manualThresholds).and(manualModeOn));

            // bindManualPivotCommand(Direction.kForward, controller.axisGreaterThan(ButtonBoardAlternate.manualPivotUp, ButtonBoardAlternate.manualThresholds).and(manualModeOn));
            // bindManualPivotCommand(Direction.kReverse, controller.axisGreaterThan(ButtonBoardAlternate.manualPivotDown, ButtonBoardAlternate.manualThresholds).and(manualModeOn));

            bindManualAlgaeCommand(AlgaeLocationPresets.GROUND, controller.pov(ButtonBoardAlternate.groundAlgae).and(algaeOn).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFLOWER, controller.button(ButtonBoardAlternate.lowAlgae).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFUPPER, controller.button(ButtonBoardAlternate.highAlgae).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.NET, controller.pov(ButtonBoardAlternate.net).and(algaeOn).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.PROCESSOR, controller.pov(ButtonBoardAlternate.processor).and(algaeOn).and(manualModeOn));

            bindManualClimbCommand(controller.button(ButtonBoardAlternate.climb).and(manualModeOn));
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

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private void configureAutonChooser() {
        if (AutonConstants.useSuperAuton) {
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
        } else {
            boolean isCompetition = false;

            // Filter any autos with the "comp" prefix
            autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream
            );
    
            autoChooser = AutoBuilder.buildAutoChooser();
    
            SmartDashboard.putData("Auto Chooser", autoChooser);
        }
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
        if (AutonConstants.useSuperAuton) {
            Pose2d[] locations = new Pose2d[scoringLocationsChooser.size()];
            for (int i = 0; i < scoringLocationsChooser.size(); i++) {
                locations[i] = scoringLocationsChooser.get(i).getSelected();
            }
    
            Command[] heights = new Command[scoringHeightsChooser.size()];
            for (int i = 0; i < scoringHeightsChooser.size(); i++) {
                heights[i] = scoringHeightsChooser.get(i).getSelected().get();
            }
    
            return AutonomousUtil.generateRoutineWithCommands(m_drivetrain, pickupLocation.getSelected(), locations, heights, this::coralIntakeCommand);
        } else {
            return autoChooser.getSelected();
        }
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("L1", coralScoreCommand(1));
        NamedCommands.registerCommand("L2", coralScoreCommand(2));
        NamedCommands.registerCommand("L3", coralScoreCommand(3));
        NamedCommands.registerCommand("L4", coralScoreCommand(4));
        NamedCommands.registerCommand("Algae Lower", algaeIntakeCommand(AlgaeLocationPresets.REEFLOWER));
        NamedCommands.registerCommand("Algae Upper", algaeIntakeCommand(AlgaeLocationPresets.REEFUPPER));
        NamedCommands.registerCommand("Processor", algaeScoreCommand(AlgaeLocationPresets.PROCESSOR));
    }

    private void configureVision() {
        if (VisionConstants.enableVision) {
            m_vision.startThread();
            RobotObserver.setSingleTagRunnable(m_vision::setSingleTag);
            RobotObserver.setMultiTagRunnable(m_vision::setMultitag);
        } else {
            m_logger.warn("Disabled vision temporarily");
        }
    }

    // ********** SUBSYSTEMS **********

    private Elevator m_elevator;
    private Pivot m_algaePivot;
    private Climber m_climber;
    private AlgaeRollers m_algaeRollers;
    private CoralRollers m_coralRollers;

    private void configureSubsystems() {
        // m_drivetrain.registerTelemetry(m_telemetry::telemeterize);
        m_elevator = new Elevator();
        m_algaePivot = new Pivot();
        m_climber = new Climber();
        m_algaeRollers = new AlgaeRollers();
        m_coralRollers = new CoralRollers();
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
            case LEFT -> trigger.whileTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(m_drivetrain, () -> coralScoreCommand(level), scoringLocationsListLeft)));
            case RIGHT -> trigger.whileTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(m_drivetrain, () -> coralScoreCommand(level), scoringLocationsRightList)));
        }   
    }

    private void bindManualCoralScoreCommand(int level, Trigger trigger) {
        trigger.whileTrue(coralScoreCommand(level));    
    }

    private void bindAutoAlgaeCommand(AlgaeLocationPresets type, Trigger trigger) {
        switch (type) {
            case GROUND -> trigger.whileTrue(algaeIntakeCommand(AlgaeLocationPresets.GROUND));
            case REEFLOWER -> trigger.whileTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(m_drivetrain, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFLOWER), scoringLocationsMiddleList)));
            case REEFUPPER -> trigger.whileTrue(new InstantCommand(() -> AutonomousUtil.queueClosest(m_drivetrain, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFUPPER), scoringLocationsMiddleList)));
            case NET -> trigger.whileTrue(new InstantCommand(() -> AutonomousUtil.queuePathWithCommand(m_drivetrain, ScoringLocations.NET.value, () -> algaeScoreCommand(AlgaeLocationPresets.PROCESSOR))));
            case PROCESSOR -> trigger.whileTrue(new InstantCommand(() -> AutonomousUtil.queuePathWithCommand(m_drivetrain, ScoringLocations.PROCESSOR.value, () -> algaeScoreCommand(AlgaeLocationPresets.PROCESSOR))));
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
            case kForward -> trigger.whileTrue(new ManualElevatorCommand(m_elevator, true));
            case kReverse -> trigger.whileTrue(new ManualElevatorCommand(m_elevator, false));
        }
    }

    private void bindManualPivotCommand(Direction direction, Trigger trigger) {
        switch (direction) {
            case kForward -> trigger.whileTrue(new ManualPivotCommand(m_algaePivot, true));
            case kReverse -> trigger.whileTrue(new ManualPivotCommand(m_algaePivot, false));
        }
    }

    private void bindManualClimbCommand(Trigger trigger) {
        trigger.whileTrue(new ManualClimberCommand(m_climber));
    }

    // ** SUBSYSTEM PASS IN HELPERS **

    private Command coralIntakeCommand() {
        return new CoralIntakeCommand(m_coralRollers, m_elevator);
    }

    private Command coralScoreCommand(int level) {
        return new CoralScoreCommand(m_coralRollers, m_elevator, level);
    }

    private Command algaeIntakeCommand(AlgaeLocationPresets intakeLocation) {
        return new AlgaeIntakeCommand(m_algaeRollers, m_elevator, m_algaePivot, intakeLocation);
    }

    private Command algaeScoreCommand(AlgaeLocationPresets scoreLocation) {
        return new AlgaeScoreCommand(m_algaeRollers, m_elevator, m_algaePivot, scoreLocation);
    }

    public enum AlgaeLocationPresets {
        REEFLOWER, REEFUPPER, PROCESSOR, GROUND, NET;
    }

    public void resetReferences() {
        m_elevator.setPosition(m_elevator.getPosition());
        m_algaePivot.setStow();
    }
}
