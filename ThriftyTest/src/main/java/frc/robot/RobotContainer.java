// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.ButtonBindingConstants;
import frc.robot.Constants.ButtonBindingConstants.DragonReins;
import frc.robot.algaeTracking.AlgaeTracker;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.TrackAlgae;
import frc.robot.generated.TestBotTunerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final Logger m_logger = LoggerFactory.getLogger(RobotContainer.class);

    private final Telemetry m_telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public final CommandSwerveDrivetrain m_drivetrain = TestBotTunerConstants.createDrivetrain();

    private final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

    public RobotContainer() {
        configureDriverBindings();
        configureTesting();
        confiureSimulation();
        SmartDashboard.putData("follow algae", new TrackAlgae(m_drivetrain, new AlgaeTracker("Cam")));
    }

    private void confiureSimulation() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void enablePDPSwitch() {
        pdp.setSwitchableChannel(true);
    }

    private void configureTesting() {
    }

    // ********** BINDINGS **********

    private void configureDriverBindings() {
        CommandPS5Controller controller = new CommandPS5Controller(ButtonBindingConstants.driverPort);

        double flipX;
        double flipY;
        double flipR;

        flipX = DragonReins.flipX ? -1.0 : 1.0;
        flipY = DragonReins.flipY ? -1.0 : 1.0;
        flipR = DragonReins.flipRot ? -1.0 : 1.0;

        Supplier<Double> xSup = () -> controller.getRawAxis(DragonReins.xAxis) * flipX;
        Supplier<Double> ySup = () -> controller.getRawAxis(DragonReins.yAxis) * flipY;
        Supplier<Double> rSup = () -> controller.getRawAxis(DragonReins.rotAxis) * flipR;

        m_drivetrain.setDefaultCommand(
            new TeleopCommand(m_drivetrain, xSup, ySup, rSup)
        );

        controller.button(DragonReins.resetHeading).onTrue(m_drivetrain.runOnce(() -> m_drivetrain.resetHeading()));
        controller.button(DragonReins.resetHeading).onFalse(m_drivetrain.runOnce(() -> m_drivetrain.resetHeading()));
    }

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
