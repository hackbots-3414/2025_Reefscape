// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    // private static final Logger log = LoggerFactory.getLogger(RobotContainer.class);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    private final ArrayList<SendableChooser<Pose2d>> scoringLocationsChooser = new ArrayList<>();
    private final SendableChooser<Pose2d> pickupLocation = new SendableChooser<>();

    private final int numAutonWaypoints = 5;
    

    public RobotContainer() {
        configureBindings();

        // boolean isCompetition = true;

        // Filter any autos with the "comp" prefix
        // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        // (stream) -> isCompetition
        //     ? stream.filter(auto -> auto.getName().startsWith("comp"))
        //     : stream
        // );

        autoChooser = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auto Chooser", autoChooser);

        for (int i = 0; i < numAutonWaypoints; i++) {
            SendableChooser<Pose2d> currChooser = new SendableChooser<>();
            configureSendableChooser(currChooser);
            scoringLocationsChooser.add(currChooser);
            SmartDashboard.putData("Piece to Score #" + (i), currChooser);
        }        

        pickupLocation.addOption("UPPER", ScoringLocations.UPPERHP.value);
        pickupLocation.addOption("LOWER", ScoringLocations.LOWERHP.value);
        SmartDashboard.putData(pickupLocation);
    }

    private void configureSendableChooser(SendableChooser<Pose2d> chooser) {
        chooser.addOption("A", ScoringLocations.A.value);
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

    public Command getAutonomousCommand() {
        Pose2d[] locations = new Pose2d[scoringLocationsChooser.size()];
        for (int i = 0; i < scoringLocationsChooser.size(); i++) {
            locations[i] = scoringLocationsChooser.get(i).getSelected();
        }
        return drivetrain.generateAutonomousRoutineFromPoses(pickupLocation.getSelected(), locations);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.button(1).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.A.value)));
        joystick.button(2).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.B.value)));
        joystick.button(3).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.C.value)));
        joystick.button(4).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.D.value)));
        joystick.button(5).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.E.value)));
        joystick.button(6).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.F.value)));
        joystick.button(7).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.G.value)));
        joystick.button(8).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.H.value)));
        joystick.button(9).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.I.value)));
        joystick.button(10).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.J.value)));
        joystick.button(11).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.K.value)));
        joystick.button(12).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.L.value)));
        joystick.button(13).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.LOWERHP.value)));
        joystick.button(14).onTrue(new InstantCommand(() -> drivetrain.sequenceOnTheFlyPaths(ScoringLocations.UPPERHP.value)));

        joystick.axisMagnitudeGreaterThan(0, 0.0).or(() -> joystick.axisMagnitudeGreaterThan(1, 0.0).getAsBoolean()).onTrue(new InstantCommand(() -> drivetrain.resetOnTheFly()));
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
        LOWERHP(new Pose2d(1.194, 1.026, Rotation2d.fromDegrees(55))),
        UPPERHP(new Pose2d(1.217, 7.012, Rotation2d.fromDegrees(-55)));
        
        private Pose2d value;

        private ScoringLocations(Pose2d value) {
            this.value = value;
        }
    }
}
