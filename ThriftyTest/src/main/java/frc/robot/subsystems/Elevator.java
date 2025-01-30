// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.stateSpace.StateSpaceController;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorLeft = new TalonFX(ElevatorConstants.leftMotorID);
    private final TalonFX elevatorRight = new TalonFX(ElevatorConstants.rightMotorID);
    private final CANcoder cancoder = new CANcoder(ElevatorConstants.encoderPort);

    private final DigitalInput forwardLimiter = new DigitalInput(ElevatorConstants.forwardLimitChannelID);
    private final DigitalInput reverseLimiter = new DigitalInput(ElevatorConstants.reverseLimitChannelID);

    private final StateSpaceController<N2, N1, N2> controller;

    private double position;
    private double velocity;
    private boolean reverseLimit;
    private boolean forwardLimit;

    private ElevatorSim elevatorSim;
    private double simPosition = 0.0; // Simulated position in meters
    private double simVelocity = 0.0; // Simulated velocity in meters per second
    private final DCMotor elevatorGearbox = DCMotor.getFalcon500(2); // 2 motors (left and right)

    private Mechanism2d mechVisual;
    private MechanismRoot2d mechRoot;
    private MechanismLigament2d elevatorArm;

    public Elevator() {
        configEncoder();
        configMotor();
        Vector<N2> initialState = getOutput();
        controller = new StateSpaceController<>(ElevatorConstants.stateSpaceConfig, this::getOutput, this::applyInput,
                initialState);
        configSim();
    }

    public void configEncoder() {
        cancoder.clearStickyFaults();
        cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);
        cancoder.getConfigurator().apply(ElevatorConstants.encoderConfig, 0.2);
    }

    public void configMotor() {
        elevatorLeft.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
        elevatorRight.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
    }

    public void configSim() {
        elevatorSim = new ElevatorSim(
                ElevatorConstants.stateSpacePlant,
                elevatorGearbox,
                ElevatorConstants.reverseSoftLimit,
                ElevatorConstants.forwardSoftLimit,
                true,
                ElevatorConstants.reverseSoftLimit
        );

        // elevatorSim = new ElevatorSim(TalonFXConstants.TalonFXDCMotor, ElevatorConstants.gearRatio, ElevatorConstants.carriageMass, ElevatorConstants.drumRadius, ElevatorConstants.reverseSoftLimit, ElevatorConstants.forwardSoftLimit, true, ElevatorConstants.reverseSoftLimit);

        mechVisual = new Mechanism2d(1, 5.0); // Width/height in meters
        mechRoot = mechVisual.getRoot("ElevatorRoot", 0.5, 0.0); // Center at (0.5, 0)
        elevatorArm = mechRoot.append(new MechanismLigament2d("ElevatorArm", 0.1, 90)); // Start at 0.1m height
        SmartDashboard.putData("Elevator Visualization", mechVisual);
    }

    private Vector<N2> getOutput() {
        if (RobotBase.isSimulation()) {
            return VecBuilder.fill(simPosition, simVelocity);
        } else {
            return VecBuilder.fill(position, velocity);
        }
    }

    private void applyInput(Vector<N1> inputs) {
        VoltageOut config = new VoltageOut(0);
        Follower follower = new Follower(ElevatorConstants.leftMotorID, ElevatorConstants.invertRightMotor);
        double volts = inputs.get(0);

        if (volts < 0) {
            config.withLimitReverseMotion(reverseLimit);
        } else if (volts > 0) {
            config.withLimitForwardMotion(forwardLimit);
        }
        elevatorLeft.setControl(config.withOutput(volts));
        elevatorRight.setControl(follower);
    }

    public void setPosition(double goal) {
        controller.setReference(VecBuilder.fill(goal, 0.0));
    }

    public void setSpeed(double speed) {
        Follower follower = new Follower(ElevatorConstants.leftMotorID, ElevatorConstants.invertRightMotor);
        elevatorLeft.setControl(new DutyCycleOut(speed));
        elevatorRight.setControl(follower);
    }

    public void enableStateSpace() {
        controller.enableStateSpace();
    }

    public void disableStateSpace() {
        controller.disableStateSpace();
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

    public void setNet() {
        setPosition(ElevatorConstants.net);
    }

    public void stop() {
        setPosition(position);
    }

    public double getPosition() {
        return position;
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
        return controller.isAtSetpoint();
    }

    @Override
    public void periodic() {
        position = cancoder.getPosition().getValueAsDouble();
        velocity = cancoder.getVelocity().getValueAsDouble();

        reverseLimit = !reverseLimiter.get();
        forwardLimit = !forwardLimiter.get();

        elevatorArm.setLength(simPosition + 0.1); // Offset to avoid overlapping with root
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation with the motor voltage
        double appliedVolts = elevatorLeft.get() * RobotController.getBatteryVoltage();
        elevatorSim.setInput(appliedVolts);
        elevatorSim.update(0.02); // Update every 20ms (standard loop time)

        // Update simulated position and velocity
        simPosition = elevatorSim.getPositionMeters();
        simVelocity = elevatorSim.getVelocityMetersPerSecond();

        // Update the simulated encoder values
        cancoder.getSimState().setRawPosition(simPosition / ElevatorConstants.metersPerRotation);
        cancoder.getSimState().setVelocity(simVelocity / ElevatorConstants.mpsPerRPM);

        // Simulate battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
}
