// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.utils.StateSpaceController;

public class Coral extends SubsystemBase {
    private TalonFX coral = new TalonFX(ElevatorConstants.motorID);

    private StateSpaceController<N2, N1, N2> controller;

    private double velocity;

    public Coral() {
        configMotor();
        Vector<N2> initialState = getOutput();
        controller = new StateSpaceController<N2, N1, N2>(ElevatorConstants.k_config, this::getOutput, this::applyInput,
                initialState);
    }

    public void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(ElevatorConstants.invertedValue))

                .withFeedback(new FeedbackConfigs()
                        .withRotorToSensorRatio(ElevatorConstants.rotorToSensorRatio)
                        .withSensorToMechanismRatio(ElevatorConstants.sensorToMechanismRatio))

                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(ElevatorConstants.supplyCurrentLimit));

        coral.getConfigurator().apply(config, 0.2);
    }

    private Vector<N2> getOutput() {
        double velocity = coral.getVelocity().getValueAsDouble(); // position is in mechanism rotations
        double acceleration = coral.getAcceleration().getValueAsDouble();
        return VecBuilder.fill(velocity, acceleration);
    }

    private void applyInput(Vector<N1> inputs) {
        VoltageOut config = new VoltageOut(0);
        double volts = inputs.get(0);

        coral.setControl(config.withOutput(volts));
    }

    public void setVelocity(double goal) {
        controller.setReference(VecBuilder.fill(goal, 0.0));
    }

    public void stop() {
        setVelocity(velocity);
    }

    public double getVelocity() {
        return velocity;
    }

    public boolean atSetpoint() {
        return velocity - controller.getReference() < PivotConstants.atSetpointTolerance;
    }

    @Override
    public void periodic() {
        velocity = coral.getVelocity().getValueAsDouble();
    }
}
