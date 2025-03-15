// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.commands.AlgaeEjectCommand;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeScoreCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CoralEjectCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralScoreCommand;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.OpenFunnel;
import frc.robot.commands.PitClimbSetupCommand;
import frc.robot.commands.ProcessorCommand;
import frc.robot.commands.StowCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedFeedback;
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
        configureTesting();
        configureDashboard();
        confiureSimulation();
    }

    private void confiureSimulation() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }
    
    private void configureDashboard() {
        SmartDashboard.putBoolean("SAFETY MODE", false);
        SmartDashboard.putData("LIFT CLIMB", new ClimberCommand(m_climber, false));
        SmartDashboard.putData("LOWER CLIMB", new PitClimbSetupCommand(m_climber));
    }

    private void configureTesting() {
        SmartDashboard.putData("Go To B", new DriveToPointCommand(ScoringLocations.B.value, m_drivetrain));

        Pose2d point = new Pose2d(8.795, 4.906, Rotation2d.fromDegrees(15.949));
        SmartDashboard.putData("camera testing spot", new DriveToPointCommand(point, m_drivetrain));

        Pose2d point2 = new Pose2d(9.936, 4.736, Rotation2d.fromDegrees(-9.001));
        SmartDashboard.putData("camera testing spot 2", new DriveToPointCommand(point2, m_drivetrain));
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
        // controller.setRumble(RumbleType.kRightRumble, 1.0);

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
        BooleanSupplier openLoopSup = controller.button(openLoop);

        m_drivetrain.setDefaultCommand(
            new TeleopCommand(m_drivetrain, xSup, ySup, rSup, openLoopSup)
        );

        controller.button(resetHeading).onTrue(m_drivetrain.runOnce(() -> m_drivetrain.resetHeading()));
        controller.button(resetHeading).onFalse(m_drivetrain.runOnce(() -> m_drivetrain.resetHeading()));

        controller.axisMagnitudeGreaterThan(xAxis, DriveConstants.k_closedLoopOverrideToleranceTranslation)
            .or(controller.axisMagnitudeGreaterThan(yAxis, DriveConstants.k_closedLoopOverrideToleranceTranslation))
            .or(controller.axisMagnitudeGreaterThan(rAxis, DriveConstants.k_closedLoopOverrideToleranceRotation))
            .onTrue(new InstantCommand(() -> AutonomousUtil.clearQueue()));

        controller.button(13).onTrue(new InstantCommand(() -> m_drivetrain.resetPose(ScoringLocationsMiddle.GH.value)));
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
        controller.setRumble(RumbleType.kBothRumble, 1.0);

        if (ButtonBindingConstants.buttonBoardChoice == ButtonBoardChoice.BUTTONBOARD) {
            controller.button(ButtonBoard.manualModeSwitch).onChange(new InstantCommand(() -> RobotObserver.toggleManualMode()));
            BooleanSupplier manualModeOn = () -> RobotObserver.getManualMode();
            BooleanSupplier manualModeOff = () -> !RobotObserver.getManualMode();

            Trigger A = controller.button(ButtonBoard.A);
            Trigger B = controller.button(ButtonBoard.B);
            Trigger C = controller.button(ButtonBoard.C);
            Trigger D = controller.button(ButtonBoard.D);
            Trigger E = controller.button(ButtonBoard.E);
            Trigger F = controller.button(ButtonBoard.F);
            Trigger G = controller.button(ButtonBoard.G);
            Trigger H = controller.button(ButtonBoard.H);
            Trigger I = controller.button(ButtonBoard.I);
            Trigger J = controller.button(ButtonBoard.J);
            Trigger K = controller.button(ButtonBoard.K);
            Trigger L = controller.button(ButtonBoard.L);

            Trigger L1 = controller.button(ButtonBoard.L1);
            Trigger L2 = controller.button(ButtonBoard.L2);
            Trigger L3 = controller.button(ButtonBoard.L3);
            Trigger L4 = controller.button(ButtonBoard.L4);

            bindButtonBoardAuto(ScoringLocations.A, 1, A.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.B, 1, B.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.C, 1, C.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.D, 1, D.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.E, 1, E.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.F, 1, F.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.G, 1, G.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.H, 1, H.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.I, 1, I.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.J, 1, J.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.K, 1, K.and(L1).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.L, 1, L.and(L1).and(manualModeOff));
            
            bindButtonBoardAuto(ScoringLocations.A, 2, A.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.B, 2, B.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.C, 2, C.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.D, 2, D.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.E, 2, E.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.F, 2, F.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.G, 2, G.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.H, 2, H.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.I, 2, I.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.J, 2, J.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.K, 2, K.and(L2).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.L, 2, L.and(L2).and(manualModeOff));

            bindButtonBoardAuto(ScoringLocations.A, 3, A.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.B, 3, B.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.C, 3, C.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.D, 3, D.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.E, 3, E.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.F, 3, F.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.G, 3, G.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.H, 3, H.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.I, 3, I.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.J, 3, J.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.K, 3, K.and(L3).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.L, 3, L.and(L3).and(manualModeOff));

            bindButtonBoardAuto(ScoringLocations.A, 4, A.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.B, 4, B.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.C, 4, C.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.D, 4, D.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.E, 4, E.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.F, 4, F.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.G, 4, G.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.H, 4, H.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.I, 4, I.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.J, 4, J.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.K, 4, K.and(L4).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.L, 4, L.and(L4).and(manualModeOff));

            
            
            Trigger algaeLow = controller.button(ButtonBoard.lowAlgae);
            Trigger algaeHigh = controller.button(ButtonBoard.highAlgae);

            bindButtonBoardAuto(ScoringLocations.A, 5, A.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.B, 5, B.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.C, 5, C.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.D, 5, D.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.E, 5, E.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.F, 5, F.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.G, 5, G.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.H, 5, H.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.I, 5, I.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.J, 5, J.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.K, 5, K.and(algaeLow).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.L, 5, L.and(algaeLow).and(manualModeOff));

            bindButtonBoardAuto(ScoringLocations.A, 6, A.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.B, 6, B.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.C, 6, C.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.D, 6, D.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.E, 6, E.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.F, 6, F.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.G, 6, G.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.H, 6, H.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.I, 6, I.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.J, 6, J.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.K, 6, K.and(algaeHigh).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.L, 6, L.and(algaeHigh).and(manualModeOff));


            bindButtonBoardAuto(ScoringLocations.LEFTHP, 0, controller.button(ButtonBoard.leftIntake).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.RIGHTHP, 0, controller.button(ButtonBoard.rightIntake).and(manualModeOff));

            bindButtonBoardAuto(ScoringLocations.PROCESSOR, 0, controller.button(ButtonBoard.processor).and(manualModeOff));
            bindButtonBoardAuto(ScoringLocations.NET, 0, controller.button(ButtonBoard.net).and(manualModeOff));


            // MANUAL CORAL SCORE COMMANDS
            bindManualCoralScoreCommand(1, L1.and(manualModeOn));
            bindManualCoralScoreCommand(2, L2.and(manualModeOn));
            bindManualCoralScoreCommand(3, L3.and(manualModeOn));
            bindManualCoralScoreCommand(4, L4.and(manualModeOn));

            bindManualCoralIntakeCommand(controller.button(ButtonBoard.leftIntake).and(manualModeOn));

            bindManualAlgaeCommand(AlgaeLocationPresets.GROUND, controller.button(ButtonBoard.groundAlgae).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFLOWER, controller.button(ButtonBoard.lowAlgae).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFUPPER, controller.button(ButtonBoard.highAlgae).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.PROCESSOR, controller.button(ButtonBoard.processor).and(manualModeOn));
            bindManualAlgaeCommand(AlgaeLocationPresets.NET, controller.button(ButtonBoard.net).and(manualModeOn));
        
            bindAutoCancelButton(controller.button(ButtonBoard.cancelAuto));
            

            bindClimbSetupCommand(controller.button(ButtonBoard.climb));
        } else {
            controller.button(ButtonBoardAlternate.manualModeSwitch).onTrue(new InstantCommand(() -> RobotObserver.toggleManualMode()));
            BooleanSupplier manualModeOn = () -> RobotObserver.getManualMode();
            BooleanSupplier manualModeOff = () -> !RobotObserver.getManualMode();

            Trigger algaeOn = controller.button(ButtonBoardAlternate.algaeModeButton);

            controller.button(ButtonBoardAlternate.ejectCoral).whileTrue(new CoralEjectCommand(m_coralRollers, m_elevator));

            // Manual Mode Off
            bindAutoCoralScoreCommand(1, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L1).and(controller.button(ButtonBoardAlternate.leftReef)).and(manualModeOff));
            bindAutoCoralScoreCommand(2, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L2).and(controller.button(ButtonBoardAlternate.leftReef)).and(manualModeOff));
            bindAutoCoralScoreCommand(3, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L3).and(controller.button(ButtonBoardAlternate.leftReef)).and(manualModeOff));
            bindAutoCoralScoreCommand(4, ReefClipLocations.LEFT, controller.pov(ButtonBoardAlternate.L4).and(controller.button(ButtonBoardAlternate.leftReef)).and(manualModeOff));
            bindAutoCoralScoreCommand(1, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L1).and(controller.button(ButtonBoardAlternate.rightReef)).and(manualModeOff));
            bindAutoCoralScoreCommand(2, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L2).and(controller.button(ButtonBoardAlternate.rightReef)).and(manualModeOff));
            bindAutoCoralScoreCommand(3, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L3).and(controller.button(ButtonBoardAlternate.rightReef)).and(manualModeOff));
            bindAutoCoralScoreCommand(4, ReefClipLocations.RIGHT, controller.pov(ButtonBoardAlternate.L4).and(controller.button(ButtonBoardAlternate.rightReef)).and(manualModeOff));

            // Manual Mode On
            bindManualCoralScoreCommand(1, controller.pov(ButtonBoardAlternate.L1).and(manualModeOn));
            bindManualCoralScoreCommand(2, controller.pov(ButtonBoardAlternate.L2).and(manualModeOn));
            bindManualCoralScoreCommand(3, controller.pov(ButtonBoardAlternate.L3).and(manualModeOn));
            bindManualCoralScoreCommand(4, controller.pov(ButtonBoardAlternate.L4).and(manualModeOn));

            bindManualCoralIntakeCommand(controller.button(ButtonBoardAlternate.intake)/*.and(manualModeOn)*/);

            bindManualAlgaeCommand(AlgaeLocationPresets.GROUND, controller.pov(ButtonBoardAlternate.groundAlgae).and(algaeOn)/*.and(manualModeOn)*/);
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFLOWER, controller.button(ButtonBoardAlternate.lowAlgae)/*.and(manualModeOn)*/);
            bindManualAlgaeCommand(AlgaeLocationPresets.REEFUPPER, controller.button(ButtonBoardAlternate.highAlgae)/*.and(manualModeOn)*/);
            bindManualAlgaeCommand(AlgaeLocationPresets.NET, controller.pov(ButtonBoardAlternate.net).and(algaeOn)/*.and(manualModeOn)*/);
            bindManualAlgaeCommand(AlgaeLocationPresets.PROCESSOR, controller.pov(ButtonBoardAlternate.processor).and(algaeOn)/*.and(manualModeOn)*/);
            bindManualAlgaeCommand(AlgaeLocationPresets.HIGHGROUND, controller.pov(ButtonBoardAlternate.highGround).and(algaeOn)/*.and(manualModeOn)*/);

            bindClimbSetupCommand(controller.button(ButtonBoardAlternate.climbReady));
            controller.button(ButtonBoardAlternate.climb).whileTrue(new ClimberCommand(m_climber));

            controller.PS().onTrue(new StowCommand(m_elevator, m_algaePivot));
        }
    }

    public List<Pose2d> scoringLocationsListLeft;
    public List<Pose2d> scoringLocationsRightList;
    public List<Pose2d> scoringLocationsMiddleList;
    public List<Pose2d> climbLocationsList;

    // ********** AUTONOMOUS **********

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

            pickupLocation.setDefaultOption("CLOSE", ScoringLocations.LEFTHP.value);
            pickupLocation.addOption("FAR", ScoringLocations.RIGHTHP.value);
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
        NamedCommands.registerCommand("L4", coralScoreCommand(4).andThen(new WaitUntilCommand(m_elevator::atSetpoint)));
        NamedCommands.registerCommand("L3", coralScoreCommand(3).andThen(new WaitUntilCommand(m_elevator::atSetpoint)));
        NamedCommands.registerCommand("Intake", coralIntakeCommand());
        NamedCommands.registerCommand("Interrupt", new WaitUntilCommand(() -> !DriverStation.isAutonomousEnabled()));
        for (ScoringLocations location : Constants.ScoringLocations.values()) {
            String name = "Align ".concat(location.toString());
            NamedCommands.registerCommand(name, new DriveToPointCommand(location.value, m_drivetrain, true));
        }
    }

    private void configureVision() {
        if (VisionConstants.enableVision) {
            m_vision.startThread();
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
    @SuppressWarnings("unused")
    private LedFeedback m_ledFeedback;

    private void configureSubsystems() {
        // m_drivetrain.registerTelemetry(m_telemetry::telemeterize);
        m_elevator = new Elevator();
        m_algaePivot = new Pivot();
        m_climber = new Climber();
        m_algaeRollers = new AlgaeRollers();
        m_coralRollers = new CoralRollers();
        // m_ledFeedback = new LedFeedback();
    }

    // ** BUTTON BOARD HELPERS **
    private void bindManualCoralIntakeCommand(Trigger trigger) {
        trigger.whileTrue(coralIntakeCommand()); 
    }

    /**
     * @param level 1, 2, 3, 4 for coral score; 5 for algae pickup low; 6 for algae pickup high; 
     * */
    @SuppressWarnings("incomplete-switch")
    private void bindButtonBoardAuto(ScoringLocations location, int level, Trigger trigger) {
        if (level == 5) {
            switch (location) {
                case A, B -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.AB.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFLOWER)), Set.of()));
                case C, D -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.CD.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFLOWER)), Set.of()));
                case E, F -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.EF.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFLOWER)), Set.of()));
                case G, H -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.GH.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFLOWER)), Set.of()));
                case I, J -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.IJ.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFLOWER)), Set.of()));
                case K, L -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.KL.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFLOWER)), Set.of()));
            }
        } else if (level == 6) {
            switch (location) {
                case A, B -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.AB.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFUPPER)), Set.of()));
                case C, D -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.CD.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFUPPER)), Set.of()));
                case E, F -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.EF.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFUPPER)), Set.of()));
                case G, H -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.GH.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFUPPER)), Set.of()));
                case I, J -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.IJ.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFUPPER)), Set.of()));
                case K, L -> trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(ScoringLocationsMiddle.KL.value, () -> algaeIntakeCommand(AlgaeLocationPresets.REEFUPPER)), Set.of()));
            }
        } else {
            trigger.whileTrue(new DeferredCommand(() -> AutonomousUtil.pathThenRunCommand(location.value, () -> coralScoreCommand(level)), Set.of()));
        }
    }

    private void bindAutoCancelButton(Trigger trigger) {
        trigger.onTrue(new InstantCommand(() -> AutonomousUtil.clearQueue()));
    }

    private void bindAutoCoralScoreCommand(int level, ReefClipLocations location, Trigger trigger) {
        switch (location) {
            case LEFT -> trigger.whileTrue(new DeferredCommand(() -> (AutonomousUtil.closestPathThenRunCommand(() -> coralScoreCommand(level), scoringLocationsListLeft).beforeStarting(new InstantCommand(() -> RobotObserver.setReefClipLocation(ReefClipLocations.LEFT)))), Set.of()));
            case RIGHT -> trigger.whileTrue(new DeferredCommand(() -> (AutonomousUtil.closestPathThenRunCommand(() -> coralScoreCommand(level), scoringLocationsRightList).beforeStarting(new InstantCommand(() -> RobotObserver.setReefClipLocation(ReefClipLocations.RIGHT)))), Set.of()));
        }   
    }

    private Command pathPlannerOverrideScore(int level, ReefClipLocations location) {
        Command c = new InstantCommand();
        switch (location) {
            case LEFT -> c = new DeferredCommand(() -> (AutonomousUtil.closestPathThenRunCommand(() -> coralScoreCommand(level), scoringLocationsListLeft).beforeStarting(new InstantCommand(() -> RobotObserver.setReefClipLocation(ReefClipLocations.LEFT)))), Set.of());
            case RIGHT -> c = new DeferredCommand(() -> (AutonomousUtil.closestPathThenRunCommand(() -> coralScoreCommand(level), scoringLocationsRightList).beforeStarting(new InstantCommand(() -> RobotObserver.setReefClipLocation(ReefClipLocations.RIGHT)))), Set.of());
        }
        return c;
    }

    private void bindManualCoralScoreCommand(int level, Trigger trigger) {
        trigger.whileTrue(coralScoreCommand(level));    
    }

    private void bindManualAlgaeCommand(AlgaeLocationPresets type, Trigger trigger) {
        switch (type) {
            case GROUND, REEFLOWER, REEFUPPER, HIGHGROUND -> trigger.whileTrue(algaeIntakeCommand(type));
            case NET -> trigger.whileTrue(algaeScoreCommand(type));
            case PROCESSOR -> {
                trigger.whileTrue(processorCommand());
                trigger.onFalse(new AlgaeEjectCommand(m_algaeRollers).andThen(new WaitCommand(2).andThen(new InstantCommand(() -> {m_algaeRollers.stopMotor(); m_algaePivot.setStow();}))));
            }
        }
    }

    private void bindClimbSetupCommand(Trigger trigger) {
        trigger.whileTrue(new SequentialCommandGroup(
            new OpenFunnel(m_climber),
            new ClimberCommand(m_climber, false)
        ));
    }

    // ** SUBSYSTEM PASS IN HELPERS **

    private Command coralIntakeCommand() {
        return new CoralIntakeCommand(m_coralRollers, m_elevator);
    }

    private Command coralScoreCommand(int level) {
        return new CoralScoreCommand(m_coralRollers, m_elevator, level).andThen(new WaitUntilCommand(m_elevator::atSetpoint));
    }

    private Command algaeIntakeCommand(AlgaeLocationPresets intakeLocation) {
        return new AlgaeIntakeCommand(m_algaeRollers, m_elevator, m_algaePivot, intakeLocation);
    }

    private Command algaeScoreCommand(AlgaeLocationPresets scoreLocation) {
        return new AlgaeScoreCommand(m_algaeRollers, m_elevator, m_algaePivot, scoreLocation);
    }

    private Command processorCommand() {
        return new ProcessorCommand(m_elevator, m_algaeRollers, m_algaePivot);
    }

    public enum AlgaeLocationPresets {
        REEFLOWER, REEFUPPER, PROCESSOR, GROUND, NET, HIGHGROUND;
    }

    public void resetReferences() {
        m_elevator.setPosition(m_elevator.getPosition());
        m_algaePivot.setStow();
    }
}
