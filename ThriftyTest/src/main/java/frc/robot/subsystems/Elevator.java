// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.StateSpaceController;

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

    public Elevator() {
        configEncoder();
        configMotor();
        Vector<N2> initialState = getOutput();
        controller = new StateSpaceController<>(ElevatorConstants.stateSpaceConfig, this::getOutput, this::applyInput,
                initialState);
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

    private Vector<N2> getOutput() {
        return VecBuilder.fill(position, velocity);
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
    }
}
