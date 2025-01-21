// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Score;
import frc.robot.commands.TeleopCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.AutonomousUtil;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 3% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // private final SendableChooser<Command> autoChooser;

    private final ArrayList<SendableChooser<Pose2d>> scoringLocationsChooser = new ArrayList<>();
    private final SendableChooser<Pose2d> pickupLocation = new SendableChooser<>();
    private final ArrayList<SendableChooser<Supplier<Command>>> scoringHeightsChooser = new ArrayList<>();

    private final int numAutonWaypoints = 5;

    private Elevator elevator;
    private Coral coral;
    

    public RobotContainer() {
        configureBindings();

        this.elevator = new Elevator();
        this.coral = new Coral();

        // boolean isCompetition = true;

        // Filter any autos with the "comp" prefix
        // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        // (stream) -> isCompetition
        //     ? stream.filter(auto -> auto.getName().startsWith("comp"))
        //     : stream
        // );

        // autoChooser = AutoBuilder.buildAutoChooser();
        
        // SmartDashboard.putData("Auto Chooser", autoChooser);

        configureAutonChooser();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            new TeleopCommand(drivetrain, this::getX, this::getY, this::getRot, this::getClosedLoopEnableButton)
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.button(1).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.A.value)));
        joystick.button(2).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.B.value)));
        joystick.button(3).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.C.value)));
        joystick.button(4).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.D.value)));
        joystick.button(5).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.E.value)));
        joystick.button(6).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.F.value)));
        joystick.button(7).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.G.value)));
        joystick.button(8).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.H.value)));
        joystick.button(9).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.I.value)));
        joystick.button(10).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.J.value)));
        joystick.button(11).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.K.value)));
        joystick.button(12).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.L.value)));
        joystick.button(13).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.FARHP.value)));
        joystick.button(14).and(joystick.button(15)).onTrue(new InstantCommand(() -> AutonomousUtil.queuePath(ScoringLocations.CLOSEHP.value)));

        joystick.button(15).onFalse(new InstantCommand(() -> AutonomousUtil.clearQueue())); // code where u can only ever queue paths while button is held, and when let go, queue will clear
        joystick.axisMagnitudeGreaterThan(0, 0.0).or(() -> joystick.axisMagnitudeGreaterThan(1, 0.0).getAsBoolean()).onTrue(new InstantCommand(() -> AutonomousUtil.clearQueue())); // can queue paths whenever, so long as no joystick input is there
    }

    private double getX() {return -joystick.getLeftY();}
    private double getY() {return -joystick.getLeftX();}
    private double getRot() {return -joystick.getRightX();}
    private boolean getClosedLoopEnableButton() {return joystick.povUp().getAsBoolean();}

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

        pickupLocation.addOption("CLOSE", ScoringLocations.CLOSEHP.value);
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


    private Command scoreL1() {return makeScoreCommand(1);};
    private Command scoreL2() {return makeScoreCommand(2);};
    private Command scoreL3() {return makeScoreCommand(3);};
    private Command scoreL4() {return makeScoreCommand(4);};
    private Command makeScoreCommand(int level) {return new Score(level, elevator, coral);}

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
