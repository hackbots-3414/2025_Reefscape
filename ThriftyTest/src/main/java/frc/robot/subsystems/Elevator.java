// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.robot.Constants.CanRangeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.stateSpace.StateSpaceController;

public class Elevator extends SubsystemBase {
    private final TalonFX m_elevatorLeft = new TalonFX(ElevatorConstants.leftMotorID);
    private final TalonFX m_elevatorRight = new TalonFX(ElevatorConstants.rightMotorID);

    private final CANcoder m_cancoder = new CANcoder(ElevatorConstants.encoderPort);

    private final CANrange m_canrange = new CANrange(CanRangeConstants.k_canRangeId);

    // Although I would love to implement a Kalman Filter for this, that takes too much time!!!
    private final MedianFilter m_filter = new MedianFilter(CanRangeConstants.k_filterWindow);

    private final DigitalInput m_forwardLimiter = new DigitalInput(ElevatorConstants.forwardLimitChannelID);
    private final DigitalInput m_reverseLimiter = new DigitalInput(ElevatorConstants.reverseLimitChannelID);

    private StateSpaceController<N2, N1, N2> m_controller;


    private double m_position;
    private double m_velocity;
    private boolean m_reverseLimit;
    private boolean m_forwardLimit;

    private boolean m_stateSpaceEnabled;

    private ElevatorSim m_elevatorSim;
    private double m_simPosition = 0.0; // Simulated position in meters
    private double m_simVelocity = 0.0; // Simulated velocity in meters per second
    private final DCMotor m_elevatorGearbox = DCMotor.getFalcon500(2); // 2 motors (left and right)

    private Mechanism2d m_mechVisual;
    private MechanismRoot2d m_mechRoot;
    private MechanismLigament2d m_elevatorArm;

    private double m_speed;
    private boolean m_speedChanged;

    public Elevator() {
        configEncoder();
        configMotor();
        configStateSpace();
        configSim();
        configCanRange();
    }

    private void configEncoder() {
        m_cancoder.clearStickyFaults();
        m_cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);
        m_cancoder.getConfigurator().apply(ElevatorConstants.encoderConfig, 0.2);
    }

    private void configMotor() {
        m_elevatorLeft.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
        m_elevatorRight.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
        Follower follower = new Follower(ElevatorConstants.leftMotorID, ElevatorConstants.invertRightMotor);
        m_elevatorRight.setControl(follower);
    }

    private void configStateSpace() {
        Vector<N2> initialState = getOutput();
        m_controller = new StateSpaceController<>(ElevatorConstants.stateSpaceConfig, this::getOutput, this::applyInput,
                initialState);
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

        m_mechVisual = new Mechanism2d(1, 5.0); // Width/height in meters
        m_mechRoot = m_mechVisual.getRoot("ElevatorRoot", 0.5, 0.0); // Center at (0.5, 0)
        m_elevatorArm = m_mechRoot.append(new MechanismLigament2d("ElevatorArm", 0.1, 90)); // Start at 0.1m height
        SmartDashboard.putData("Elevator Visualization", m_mechVisual);
    }

    private void configCanRange() {
        m_canrange.clearStickyFaults();
        m_canrange.getConfigurator().apply(
            CanRangeConstants.k_canRangeConfig,
            RobotConstants.globalCanTimeout.in(Seconds)
        );
    }

    private Vector<N2> getOutput() {
        if (RobotBase.isSimulation()) {
            return VecBuilder.fill(m_simPosition, m_simVelocity);
        } else {
            return VecBuilder.fill(m_position, m_velocity);
        }
    }

    private void applyInput(Vector<N1> inputs) {
        if (!m_stateSpaceEnabled) return;

        VoltageOut config = new VoltageOut(0);
        double volts = inputs.get(0);

        if (volts < 0) {
            config.withLimitReverseMotion(m_reverseLimit);
        } else if (volts > 0) {
            config.withLimitForwardMotion(m_forwardLimit);
        }
        m_elevatorLeft.setControl(config.withOutput(volts));
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

    @Override
    public void periodic() {
        m_velocity = m_cancoder.getVelocity().getValueAsDouble();

        m_reverseLimit = !m_reverseLimiter.get();
        m_forwardLimit = !m_forwardLimiter.get();

        m_elevatorArm.setLength(m_simPosition + 0.1); // Offset to avoid overlapping with root

        m_position = m_filter.calculate(m_canrange.getDistance().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Position", m_position);

        if (m_speedChanged && !m_stateSpaceEnabled) {
            m_elevatorLeft.setControl(new DutyCycleOut(m_speed));
            m_speedChanged = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation with the motor voltage
        double appliedVolts = m_elevatorLeft.get() * RobotController.getBatteryVoltage();
        m_elevatorSim.setInput(appliedVolts);
        m_elevatorSim.update(0.02); // Update every 20ms (standard loop time)

        // Update simulated position and velocity
        m_simPosition = m_elevatorSim.getPositionMeters();
        m_simVelocity = m_elevatorSim.getVelocityMetersPerSecond();

        // Update the simulated encoder values
        m_cancoder.getSimState().setRawPosition(m_simPosition);
        m_cancoder.getSimState().setVelocity(m_simVelocity);

        // Simulate battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }
}
