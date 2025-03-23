// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.RobotObserver;

public class Elevator extends SubsystemBase {
    // we want to have a logger, even if we're not using it... yet
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(Elevator.class);

    private final TalonFX m_elevatorLeft = new TalonFX(IDConstants.elevatorLeft, "*");
    private final TalonFX m_elevatorRight = new TalonFX(IDConstants.elevatorRight, "*");

    private final CANcoder m_cancoder = new CANcoder(IDConstants.elevatorEncoder);

    private double m_position;
    private double m_velocity;

    private double m_reference;

    private double m_compensation;

    private ElevatorSim m_elevatorSim;
    private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60(2); // 2 motors (left and right)

    private Mechanism2d m_mechVisual;
    private MechanismRoot2d m_mechRoot;
    private MechanismLigament2d m_elevatorArm;

    private double m_speed;
    private boolean m_speedChanged;

    public Elevator() {
        configEncoder();
        configMotor();
        configSim();
        SmartDashboard.putData("Zero Elevator", new InstantCommand(this::zeroElevator).ignoringDisable(true));
    }

    private void configEncoder() {
        m_cancoder.clearStickyFaults();
        m_cancoder.getConfigurator().apply(ElevatorConstants.encoderConfig, 0.2);
    }

    private void configMotor() {
        m_elevatorRight.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
        m_elevatorLeft.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
        Follower follower = new Follower(IDConstants.elevatorRight, ElevatorConstants.invertLeftMotorFollower);
        m_elevatorLeft.setControl(follower);
    }

    private void configSim() {
        m_elevatorSim = new ElevatorSim(
                ElevatorConstants.stateSpacePlant,
                m_elevatorGearbox,
                ElevatorConstants.reverseSoftLimit,
                ElevatorConstants.forwardSoftLimit,
                true,
                ElevatorConstants.reverseSoftLimit
        );

        m_mechVisual = new Mechanism2d(1, 12); // Width/height in meters
        m_mechRoot = m_mechVisual.getRoot("ElevatorRoot", 0.5, 0.0); // Center at (0.5, 0)
        m_elevatorArm = m_mechRoot.append(new MechanismLigament2d("ElevatorArm", 0.0, 90)); // Start at 0.1m height
        SmartDashboard.putData("Elevator Visualization", m_mechVisual);
        if (RobotBase.isSimulation()) {
            // in simulation, we want to emulate the effect produced by
            // using an encoder offset (i.e. we start at 0).
            m_elevatorRight.setPosition(0.0);
        }
    }

    // private final MotionMagicVoltage control = new MotionMagicVoltage(0);
    private final DynamicMotionMagicVoltage control = new DynamicMotionMagicVoltage(0, 0, 0, 0);

    public void setPosition(double goal) {
        // floor values for the goal between our two extrema for their positions
        goal = Math.min(goal, ElevatorConstants.forwardSoftLimit);
        goal = Math.max(goal, ElevatorConstants.reverseSoftLimit);
        m_logger.info("Setpoint is: {}", goal);
        if (goal >= getPosition()) {
            m_elevatorRight.setControl(control
                .withPosition(goal)
                .withVelocity(ElevatorConstants.maxSpeedUp)
                .withAcceleration(ElevatorConstants.maxSpeedUp * ElevatorConstants.accelerationMultiplierUp)
                .withJerk(ElevatorConstants.maxSpeedUp * ElevatorConstants.accelerationMultiplierUp * 10)
                .withSlot(0));
        } else {
            m_elevatorRight.setControl(control
                .withPosition(goal)
                .withVelocity(ElevatorConstants.maxSpeedDown)
                .withAcceleration(ElevatorConstants.maxSpeedDown * ElevatorConstants.accelerationMultiplierUp)
                .withJerk(ElevatorConstants.maxSpeedDown * ElevatorConstants.accelerationMultiplierUp * 10)
                .withSlot(1));
        }

        // m_elevatorRight.setControl(control.withPosition(goal));
        m_reference = goal;
    }

    public void setSpeed(double speed) {
        m_speedChanged = (speed != m_speed);
        m_speed = speed;
    }

    public void setGroundIntake() {
        setPosition(ElevatorConstants.groundIntake);
    }

    public void setHighGroundIntake() {
        setPosition(ElevatorConstants.highGroundIntake);
    }

    public void setStow() {
        setPosition(ElevatorConstants.stow);
    }

    public void setProcessor() {
        setPosition(ElevatorConstants.processor);
    }

    public void setL1() {
        setPosition(ElevatorConstants.L1 + m_compensation);
    }

    public void setL2() {
        setPosition(ElevatorConstants.L2 + m_compensation);
    }

    public void setL3() {
        setPosition(ElevatorConstants.L3 + m_compensation);
    }

    public void setL4() {
        setPosition(ElevatorConstants.L4 + m_compensation);
    }

    private double getCANRangeCompensation() {
        if (RobotObserver.getManualMode() || !ElevatorConstants.enableCANRange) {
            m_logger.debug("not doing compensation");
            return 0.0;
        };
        Optional<Double> distance = RobotObserver.getCompensationDistance();
        if (distance.isEmpty()) return 0.0;
        double comp = Math.min(
            ElevatorConstants.k_maxCanCompensation,
            (distance.get() - DriveConstants.rangeZero) * ElevatorConstants.rangeDistanceGain * ElevatorConstants.inch
        );
        return comp;
    }

    public void setReefLower() {
        setPosition(ElevatorConstants.reefLower);
    }

    public void setReefUpper() {
        setPosition(ElevatorConstants.reefUpper);
    }

    public void setNet() {
        setPosition(ElevatorConstants.net);
    }

    public void stop() {
        setPosition(m_position);
    }

    public void setLevel(int level) {
        switch (level) {
            case 1 -> setL1();
            case 2 -> setL2();
            case 3 -> setL3();
            case 4 -> setL4();
            case 0 -> setStow();
            default -> setStow();
        }
    }

    public boolean atSetpoint() {
        return Math.abs(m_reference - m_position) < ElevatorConstants.tolerance;
    }

    public double getReference() {
        return m_reference;
    }

    public double getPosition() {
        return m_position;
    }

    public double getVelocity() {
        return m_velocity;
    }

    private double getPositionUncached() {
        if (RobotBase.isReal()) {
            return m_elevatorRight.getPosition().getValueAsDouble();
        } else {
            return m_elevatorSim.getPositionMeters();
        }
    }

    private double getVelocityUncached() {
        if (RobotBase.isReal()) {
            return m_elevatorRight.getVelocity().getValueAsDouble();
        } else {
            return m_elevatorSim.getVelocityMetersPerSecond();
        }
    }

    private void zeroElevator() {
        m_elevatorRight.setPosition(0.0, 0.2);
        m_elevatorLeft.setPosition(0.0, 0.2);
        m_logger.info("Cancoder Error: {}", m_cancoder.setPosition(0.0, 0.2));
    }

    @Override
    public void periodic() {
        if (ElevatorConstants.enable) {
            m_position = getPositionUncached();
            m_velocity = getVelocityUncached();

            if (m_speedChanged) {
                m_elevatorRight.setControl(new DutyCycleOut(m_speed));
                m_speedChanged = false;
            }

            SmartDashboard.putBoolean("ELEVATOR AT POSITION", atSetpoint());
            m_compensation = getCANRangeCompensation();
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation with the motor voltage
        double appliedVolts = m_elevatorRight.get() * RobotController.getBatteryVoltage() * 10;

        m_elevatorSim.setInput(appliedVolts);
        m_elevatorSim.update(SimConstants.k_simPeriodic);

        m_position = getPositionUncached();
        m_velocity = getVelocityUncached();

        // Update the simulated encoder values
        m_elevatorRight.getSimState().setRawRotorPosition(m_position);
        m_elevatorRight.getSimState().setRotorVelocity(m_velocity);

        m_elevatorArm.setLength(m_position + 0.1); // Offset to avoid overlapping with root
        
        // Simulate battery voltage
        double volts = BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps());
        RoboRioSim.setVInVoltage(volts);
    }
}
