// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ButtonBindingConstants;
import frc.robot.Constants.ButtonBindingConstants.DragonReins;
import frc.robot.Constants.ButtonBindingConstants.PS5;
import frc.robot.Constants.ClimbLocations;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ReefClipLocations;
import frc.robot.Constants.ScoringLocations;
import frc.robot.Constants.ScoringLocationsLeft;
import frc.robot.Constants.ScoringLocationsMiddle;
import frc.robot.Constants.ScoringLocationsRight;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlgaeEjectCommand;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeScoreCommand;
import frc.robot.commands.ClimbReadyCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CoralEjectCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralScoreCommand;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.commands.ElevatorToPointCommand;
import frc.robot.commands.ElevatorZero;
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
import frc.robot.utils.FieldUtils;
import frc.robot.vision.VisionHandler;

public class RobotContainer {
    private final Logger m_logger = LoggerFactory.getLogger(RobotContainer.class);

    @SuppressWarnings("unused")
    private final Telemetry m_telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    private final VisionHandler m_vision = new VisionHandler(m_drivetrain);

    private final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

    public RobotContainer() {
        configureSysId();
        configureSubsystems();
        generateScoringLocations();
        configureNamedCommands();
        configureDriverBindings();
        configureOperatorBindings();
        configureAutonChooser();
        configureVision();
        configureTesting();
        configureDashboard();
        confiureSimulation();
        RobotObserver.setFFEnabledSupplier(this::getFFEnabled);
    }

    private void configureSysId() {
        SmartDashboard.putData("quasistatic forward steer", m_drivetrain.sysIdQuasistaticSteer(Direction.kForward));
        SmartDashboard.putData("quasistatic reverse steer", m_drivetrain.sysIdQuasistaticSteer(Direction.kReverse));
        SmartDashboard.putData("dynamic forward steer", m_drivetrain.sysIdDynamicSteer(Direction.kForward));
        SmartDashboard.putData("dynamic reverse steer", m_drivetrain.sysIdDynamicSteer(Direction.kReverse));

        SmartDashboard.putData("quasistatic forward translation", m_drivetrain.sysIdQuasistaticTranslation(Direction.kForward));
        SmartDashboard.putData("quasistatic reverse translation", m_drivetrain.sysIdQuasistaticTranslation(Direction.kReverse));
        SmartDashboard.putData("dynamic forward translation", m_drivetrain.sysIdDynamicTranslation(Direction.kForward));
        SmartDashboard.putData("dynamic reverse translation", m_drivetrain.sysIdDynamicTranslation(Direction.kReverse));
    }

    public List<Pose2d> scoringLocationsListLeft;
    public List<Pose2d> scoringLocationsRightList;
    public List<Pose2d> scoringLocationsMiddleList;
    public List<Pose2d> climbLocationsList;

    private void generateScoringLocations() {
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
    }

    private void confiureSimulation() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void enablePDPSwitch() {
        pdp.setSwitchableChannel(true);
    }
    
    private void configureDashboard() {
        SmartDashboard.putData("LIFT CLIMB", new ClimberCommand(m_climber, false));
        SmartDashboard.putData("LOWER CLIMB", new PitClimbSetupCommand(m_climber));
        SmartDashboard.putData("Lazy Zero Elevator", m_elevator.runOnce(m_elevator::zeroElevator).ignoringDisable(true));
        SmartDashboard.putData("Set Center", new InstantCommand(() -> {
            m_drivetrain.setPose(FieldUtils.flipPose(new Pose2d(7.6, 4.025, Rotation2d.k180deg)));
        }).ignoringDisable(true));
    }


    private void configureTesting() {
        SmartDashboard.putData("Reset Pose", m_drivetrain.runOnce(() -> {m_drivetrain.setPose(new Pose2d(0, 0, Rotation2d.kCCW_90deg));}));
    }

    // ********** BINDINGS **********

    private void configureDriverBindings() {
        CommandPS5Controller controller = new CommandPS5Controller(ButtonBindingConstants.driverPort);
        // controller.setRumble(RumbleType.kRightRumble, 1.0);

        int xAxis;
        int yAxis;
        int rAxis; // rotation
        int resetHeading;

        double flipX;
        double flipY;
        double flipR;

        xAxis = DragonReins.xAxis;
        yAxis = DragonReins.yAxis;
        rAxis = DragonReins.rotAxis;

        resetHeading = DragonReins.resetHeading;

        flipX = DragonReins.flipX ? -1.0 : 1.0;
        flipY = DragonReins.flipY ? -1.0 : 1.0;
        flipR = DragonReins.flipRot ? -1.0 : 1.0;

        Supplier<Double> xSup = () -> controller.getRawAxis(xAxis) * flipX;
        Supplier<Double> ySup = () -> controller.getRawAxis(yAxis) * flipY;
        Supplier<Double> rSup = () -> controller.getRawAxis(rAxis) * flipR;

        m_drivetrain.setDefaultCommand(
            new TeleopCommand(m_drivetrain, xSup, ySup, rSup)
        );

        controller.button(resetHeading).onTrue(m_drivetrain.runOnce(() -> m_drivetrain.resetHeading()));
        controller.button(resetHeading).onFalse(m_drivetrain.runOnce(() -> m_drivetrain.resetHeading()));

        bindAutoProcessCommand(controller.button(DragonReins.processor));

        controller.axisMagnitudeGreaterThan(xAxis, DriveConstants.k_closedLoopOverrideToleranceTranslation)
            .or(controller.axisMagnitudeGreaterThan(yAxis, DriveConstants.k_closedLoopOverrideToleranceTranslation))
            .or(controller.axisMagnitudeGreaterThan(rAxis, DriveConstants.k_closedLoopOverrideToleranceRotation))
            .onTrue(new InstantCommand(() -> AutonomousUtil.clearQueue()));
    }

    private void configureOperatorBindings() {
        // handle bindings
        CommandPS5Controller controller = new CommandPS5Controller(ButtonBindingConstants.buttonBoardPort);

        Trigger algaeOn = controller.button(PS5.algaeModeButton);

        controller.button(PS5.ejectCoral).whileTrue(new CoralEjectCommand(m_coralRollers, m_elevator));

        bindAlignCommand(ReefClipLocations.LEFT, controller.button(PS5.leftReef));
        bindAlignCommand(ReefClipLocations.RIGHT, controller.button(PS5.rightReef));

        bindL1Command(false, controller.pov(PS5.L1).and(algaeOn.negate()).and(controller.button(PS5.secondaryL1).negate()));
        bindL1Command(true, controller.pov(PS5.L1).and(controller.button(PS5.secondaryL1)).and(algaeOn.negate()));
        bindCoralCommands(2, controller.pov(PS5.L2).and(algaeOn.negate()));
        bindCoralCommands(3, controller.pov(PS5.L3).and(algaeOn.negate()));
        bindCoralCommands(4, controller.pov(PS5.L4).and(algaeOn.negate()));

        bindCoralIntakeCommand(controller.button(PS5.intake));

        bindAlgaeIntakeCommand(AlgaeLocationPresets.REEFLOWER, controller.button(PS5.lowAlgae));
        bindAlgaeIntakeCommand(AlgaeLocationPresets.REEFUPPER, controller.button(PS5.highAlgae));
        bindAlgaeIntakeCommand(AlgaeLocationPresets.GROUND, controller.pov(PS5.groundAlgae).and(algaeOn));
        bindAlgaeIntakeCommand(AlgaeLocationPresets.HIGHGROUND, controller.pov(PS5.highGround).and(algaeOn));

        bindAlgaeScoreCommand(AlgaeLocationPresets.PROCESSOR, controller.pov(PS5.processor).and(algaeOn));
        bindAlgaeScoreCommand(AlgaeLocationPresets.NET, controller.pov(PS5.net).and(algaeOn));

        controller.button(PS5.climbReady).whileTrue(new ClimbReadyCommand(m_climber));
        controller.button(PS5.climb).whileTrue(new ClimberCommand(m_climber));

        controller.button(PS5.stow).onTrue(new StowCommand(m_elevator, m_algaePivot));

        bindFunnelOpenCommand(controller.button(11).and(controller.button(12)));

        bindElevatorZeroCommand(controller.button(PS5.zeroElevator));
    }

    private void bindElevatorZeroCommand(Trigger trigger) {
        trigger.whileTrue(zero());
    }

    private void bindL1Command(boolean higher, Trigger trigger) {
        Runnable elevatorCommand = (higher) ? m_elevator::setSecondaryL1 : m_elevator::setL1;
        trigger.whileTrue(
            m_elevator.run(elevatorCommand)
                .until(m_elevator::atSetpoint)
            .andThen(
                m_coralRollers.run(m_coralRollers::setL1Eject)
                    .onlyWhile(m_coralRollers::getCANrangeTriggered)
            )
            .finallyDo(m_coralRollers::stop)
        );
        trigger.onFalse(m_elevator.runOnce(m_elevator::release));
    }

    private void bindCoralCommands(int level, Trigger trigger) {
        trigger.whileTrue(coralPrepAndScoreCommand(level));
        trigger.onFalse(coralScoreCommand(level).andThen(zero()));
    }

    private void bindAutoProcessCommand(Trigger trigger) {
        trigger.whileTrue(new DriveToPointCommand(FieldConstants.k_processor, m_drivetrain, true));
    }

    private Command zero() {
        // return new InstantCommand();
        return new ElevatorZero(m_elevator).withTimeout(2);
    }

    // ********** AUTONOMOUS **********

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private void configureAutonChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.getSelected();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("L4", coralScoreCommand(4));
        NamedCommands.registerCommand("L3", coralScoreCommand(3));
        NamedCommands.registerCommand("Intake", m_elevator.run(m_elevator::setStow)
            .until(m_elevator::atSetpoint)
            .andThen(zero())
            .andThen(coralIntakeCommand()
                .withTimeout(5)));
        NamedCommands.registerCommand("Intake Wait", new WaitUntilCommand(m_coralRollers::intakeReady)
            .alongWith(m_drivetrain.runOnce(m_drivetrain::stop)));
        NamedCommands.registerCommand("Interrupt", new WaitUntilCommand(() -> !DriverStation.isAutonomousEnabled()));
        for (ScoringLocations location : Constants.ScoringLocations.values()) {
            String name = "Align ".concat(location.toString());
            NamedCommands.registerCommand(name, new DriveToPointCommand(location.value, m_drivetrain, true)
                .withTimeout(5.0));
        }
        NamedCommands.registerCommand("Align IJ", new DriveToPointCommand(Constants.FieldConstants.kIJ, m_drivetrain, true));
        NamedCommands.registerCommand("Align GH", new DriveToPointCommand(Constants.FieldConstants.kGH, m_drivetrain, true));
        NamedCommands.registerCommand("Align Barge", new DriveToPointCommand(FieldConstants.kBarge1, m_drivetrain, true));
        NamedCommands.registerCommand("LIntake Align", new DriveToPointCommand(FieldConstants.kLeftIntake, m_drivetrain, true)
            .until(m_coralRollers::intakeReady));
        NamedCommands.registerCommand("RIntake Align", new DriveToPointCommand(FieldConstants.kRightIntake, m_drivetrain, true)
            .until(m_coralRollers::intakeReady));
        NamedCommands.registerCommand("AlgaeUpper", algaeIntakeCommand(AlgaeLocationPresets.REEFUPPER)
            .until(m_algaeRollers::algaeHeld));
        NamedCommands.registerCommand("AlgaeLower", algaeIntakeCommand(AlgaeLocationPresets.REEFLOWER)
            .until(m_algaeRollers::algaeHeld));
        NamedCommands.registerCommand("Stop", m_drivetrain.runOnce(m_drivetrain::stop));
        NamedCommands.registerCommand("Net", algaeScoreCommand(AlgaeLocationPresets.NET)
            .andThen(zero()));
        NamedCommands.registerCommand("Process", new DriveToPointCommand(FieldConstants.k_processor, m_drivetrain, true)
            .alongWith(algaeScoreCommand(AlgaeLocationPresets.PROCESSOR)));
        NamedCommands.registerCommand("Algae End", m_algaePivot.run(m_algaePivot::setGroundPickup)
            .alongWith(m_elevator.run(m_elevator::setGroundIntake)));
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
        m_ledFeedback = new LedFeedback();
        m_elevator.setDefaultCommand(new ElevatorDefaultCommand(m_elevator));
    }

    // ** BUTTON BOARD HELPERS **
    private void bindCoralIntakeCommand(Trigger trigger) {
        trigger.whileTrue(coralIntakeCommand());
        trigger.onFalse(coralIntakeCommand().onlyWhile(m_coralRollers::presentPiece));
    }

    private void bindAlignCommand(ReefClipLocations location, Trigger trigger) {
        switch (location) {
            case LEFT -> trigger.whileTrue(new DeferredCommand(() -> (AutonomousUtil.closestPathThenRunCommand(() -> new InstantCommand(), scoringLocationsListLeft).beforeStarting(new InstantCommand(() -> RobotObserver.setReefClipLocation(ReefClipLocations.LEFT)))), Set.of()));
            case RIGHT -> trigger.whileTrue(new DeferredCommand(() -> (AutonomousUtil.closestPathThenRunCommand(() -> new InstantCommand(), scoringLocationsRightList).beforeStarting(new InstantCommand(() -> RobotObserver.setReefClipLocation(ReefClipLocations.RIGHT)))), Set.of()));
        }   
    }

    private void bindAlgaeIntakeCommand(AlgaeLocationPresets location, Trigger trigger) {
        trigger.whileTrue(algaeIntakeCommand(location).andThen(zero()));
    }

    private void bindAlgaeScoreCommand(AlgaeLocationPresets type, Trigger trigger) {
        switch (type) {
            case NET -> {
                trigger.whileTrue(m_elevator.run(m_elevator::setNet).onlyIf(m_algaeRollers::algaeHeld));
                trigger.onFalse(
                    algaeEjectCommand().onlyIf(m_elevator::atSetpoint)
                    .andThen(
                        zero()
                        .andThen(new InstantCommand(() -> {
                            m_logger.debug("zeroed elevator");
                        }))
                        .unless(RobotObserver::getNoElevatorZone)
                    ).andThen(m_elevator::release)
                    .andThen(new InstantCommand(() -> {
                        m_logger.info("released elevator");
                    }))
                );
            }
            case PROCESSOR -> {
                trigger.whileTrue(processorCommand());
                trigger.onFalse(new AlgaeEjectCommand(m_algaeRollers, m_elevator)
                    .andThen(new WaitCommand(2)
                        .andThen(new InstantCommand(() -> {
                            m_algaeRollers.stopMotor();
                            m_algaePivot.setStow();
                        }, m_algaePivot, m_algaeRollers)))
                    .andThen(zero()));
            }
            default -> {}
        }
    }

    private void bindFunnelOpenCommand(Trigger trigger) {
        trigger.whileTrue(new OpenFunnel(m_climber));
    }

    private Command algaeEjectCommand() {
        return m_algaeRollers.startEnd(m_algaeRollers::ejectAlgae, m_algaeRollers::stopMotor).onlyWhile(m_algaeRollers::algaeHeld);
    }

    // ** SUBSYSTEM PASS IN HELPERS **

    private Command coralIntakeCommand() {
        return new CoralIntakeCommand(m_coralRollers, m_elevator);
    }

    private Command coralPrepAndScoreCommand(int level) {
        return new ElevatorToPointCommand(level, m_elevator)
            .andThen(new WaitUntilCommand(m_drivetrain::isAligned))
            .andThen(coralScoreCommand(level))
            .onlyIf(m_coralRollers::holdingPiece);
    }

    private Command elevatorPrepCommand(int level) {
        return new ElevatorToPointCommand(level, m_elevator)
            .onlyIf(m_coralRollers::holdingPiece);
    }

    private Command coralScoreCommand(int level) {
        return new CoralScoreCommand(m_coralRollers, m_elevator, level)
            .andThen(new WaitUntilCommand(m_elevator::atSetpoint)
            .onlyIf(m_coralRollers::holdingPiece)
            );
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
        // m_elevator.setPosition(m_elevator.getPosition());
        m_algaePivot.setStow();
    }

    public boolean getFFEnabled() {
        return m_elevator.elevatorUp() && !DriverStation.isAutonomous();
    }
}
