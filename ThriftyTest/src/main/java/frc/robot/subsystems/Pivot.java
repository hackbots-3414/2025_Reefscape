// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.utils.StateSpaceController;

public class Pivot extends SubsystemBase {
    private final TalonFX pivot = new TalonFX(PivotConstants.motorID);
    private final CANcoder cancoder = new CANcoder(PivotConstants.encoderID);

    private final StateSpaceController<N2, N1, N2> controller;

    private double position;
    private double velocity;

    public Pivot() {
        configEncoder();
        configMotor();
        Vector<N2> initialState = getOutput();
        controller = new StateSpaceController<>(PivotConstants.stateSpaceConfig, this::getOutput, this::applyInput,
                initialState);
    }

    public void configEncoder() {
        cancoder.clearStickyFaults();
        cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);
        cancoder.getConfigurator().apply(PivotConstants.encoderConfig, 0.2);
    }

    public void configMotor() {
        pivot.getConfigurator().apply(PivotConstants.motorConfig, 0.2);
    }

    private Vector<N2> getOutput() {
        return VecBuilder.fill(position, velocity);
    }

    private void applyInput(Vector<N1> inputs) {
        VoltageOut config = new VoltageOut(0);
        double volts = inputs.get(0);

        pivot.setControl(config.withOutput(volts));
    }

    public void setPosition(double goal) {
        controller.setReference(VecBuilder.fill(goal, 0.0));
    }

    public void setStow() {
        setPosition(PivotConstants.stow);
    }

    public void setProcessor() {
        setPosition(PivotConstants.processor);
    }

    public void setNet() {
        setPosition(PivotConstants.net);
    }

    public void setGroundPickup() {
        setPosition(PivotConstants.groundPickup);
    }

    public void setReefPickup() {
        setPosition(PivotConstants.reefPickup);
    }

    public void stop() {
        setPosition(position);
    }

    public double getPosition() {
        return position;
    }

    public boolean atSetpoint() {
        return controller.isAtSetpoint();
    }

    @Override
    public void periodic() {
        position = cancoder.getPosition().getValueAsDouble();
        velocity = cancoder.getVelocity().getValueAsDouble();
    }
}
