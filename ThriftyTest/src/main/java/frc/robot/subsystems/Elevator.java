// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.stateSpace.StateSpaceController;

public class Elevator extends SubsystemBase {
    // we want to have a logger, even if we're not using it... yet
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(Elevator.class);

    private final TalonFX m_elevatorLeft = new TalonFX(IDConstants.elevatorLeft);
    private final TalonFX m_elevatorRight = new TalonFX(IDConstants.elevatorRight);

    private final CANcoder m_cancoder = new CANcoder(IDConstants.elevatorEncoder);

    private StateSpaceController<N2, N1, N2> m_controller;

    private double m_position;
    private double m_velocity;

    private boolean m_stateSpaceEnabled;

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
        configStateSpace();
    }

    private void configEncoder() {
        m_cancoder.clearStickyFaults();
        m_cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);
        m_cancoder.getConfigurator().apply(ElevatorConstants.encoderConfig, 0.2);
    }

    private void configMotor() {
        m_elevatorRight.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
        m_elevatorLeft.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
        Follower follower = new Follower(IDConstants.elevatorRight, ElevatorConstants.invertLeftMotorFollower);
        m_elevatorLeft.setControl(follower);
    }

    private void configStateSpace() {
        Vector<N2> initialState = getOutput();
        m_controller = new StateSpaceController<>(
            ElevatorConstants.stateSpaceConfig,
            this::getOutput,
            this::applyInput,
            initialState
        );
        enableStateSpace();
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

        m_mechVisual = new Mechanism2d(1, 2.0); // Width/height in meters
        m_mechRoot = m_mechVisual.getRoot("ElevatorRoot", 0.5, 0.0); // Center at (0.5, 0)
        m_elevatorArm = m_mechRoot.append(new MechanismLigament2d("ElevatorArm", 0.1, 90)); // Start at 0.1m height
        SmartDashboard.putData("Elevator Visualization", m_mechVisual);
        if (RobotBase.isSimulation()) {
            // in simulation, we want to emulate the effect produced by
            // using an encoder offset (i.e. we start at 0).
            m_cancoder.setPosition(0.0);
        }
    }

    private Vector<N2> getOutput() {
        return VecBuilder.fill(
            getPositionUncached(),
            getVelocityUncached()
        );
    }

    private void applyInput(Vector<N1> inputs) {
        if (!m_stateSpaceEnabled) return;

        double volts = inputs.get(0);

        m_elevatorRight.setVoltage(volts);
    }

    public void setPosition(double goal) {
        m_controller.setReference(VecBuilder.fill(goal, 0.0));
    }

    public void setSpeed(double speed) {
        m_speedChanged = (speed != m_speed);
        m_speed = speed;
    }

    public void enableStateSpace() {
        m_controller.setReference(getOutput());
        m_stateSpaceEnabled = true;
    }

    public void disableStateSpace() {
        m_stateSpaceEnabled = false;
    }

    public void setStow() {
        setPosition(ElevatorConstants.stow);
    }

    public void setProcessor() {
        setPosition(ElevatorConstants.processor);
    }

    public void setL1() {
        setPosition(ElevatorConstants.L1);
    }

    public void setL2() {
        setPosition(ElevatorConstants.L2);
    }

    public void setL3() {
        setPosition(ElevatorConstants.L3);
    }

    public void setL4() {
        setPosition(ElevatorConstants.L4);
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
        return m_controller.isAtSetpoint();
    }

    public double getPosition() {
        return m_position;
    }

    public double getVelocity() {
        return m_velocity;
    }

    private double getPositionUncached() {
        if (RobotBase.isReal()) {
            return m_cancoder.getPosition().getValueAsDouble();
        } else {
            return m_elevatorSim.getPositionMeters();
        }
    }

    private double getVelocityUncached() {
        if (RobotBase.isReal()) {
            return m_cancoder.getVelocity().getValueAsDouble();
        } else {
            return m_elevatorSim.getVelocityMetersPerSecond();
        }
    }

    @Override
    public void periodic() {
        m_elevatorArm.setLength(m_position + 0.1); // Offset to avoid overlapping with root

        m_position = getPositionUncached();
        m_velocity = getVelocityUncached();

        SmartDashboard.putNumber("Elevator Position", m_position);

        if (m_speedChanged && !m_stateSpaceEnabled) {
            m_elevatorRight.setControl(new DutyCycleOut(m_speed));
            m_speedChanged = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation with the motor voltage
        double appliedVolts = m_elevatorRight.get() * RobotController.getBatteryVoltage();
        double current = m_elevatorRight.getSupplyCurrent().getValueAsDouble();
        SmartDashboard.putNumber("Elevator Voltage", appliedVolts);
        SmartDashboard.putNumber("Elevator Supply Current", current);

        double speed = m_elevatorRight.get();
        SmartDashboard.putNumber("Elevator Speed", speed);

        m_elevatorSim.setInput(appliedVolts);
        m_elevatorSim.update(SimConstants.k_simPeriodic);

        m_position = getPositionUncached();
        m_velocity = getVelocityUncached();

        // Update the simulated encoder values
        m_cancoder.getSimState().setRawPosition(m_position);
        m_cancoder.getSimState().setVelocity(m_velocity);

        // Simulate battery voltage
        double volts = BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps());
        RoboRioSim.setVInVoltage(volts);
    }
}
