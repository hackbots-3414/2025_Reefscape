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
import frc.robot.utils.StateSpaceController;

public class Elevator extends SubsystemBase {
  private final TalonFX elevator = new TalonFX(ElevatorConstants.motorID);
  private final CANcoder cancoder = new CANcoder(ElevatorConstants.cancoderPort);

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
    controller = new StateSpaceController<>(ElevatorConstants.k_config, this::getOutput, this::applyInput, initialState);
  }

  public void configEncoder() {
    cancoder.clearStickyFaults();
    cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(ElevatorConstants.k_absoluteSensorRange)
            .withSensorDirection(ElevatorConstants.k_cancoderInvert)
            .withMagnetOffset(ElevatorConstants.k_encoderOffset));

    cancoder.getConfigurator().apply(canCoderConfiguration, 0.2);
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
            .withSupplyCurrentLimit(ElevatorConstants.supplyCurrentLimit))

        .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
            .withForwardLimitEnable(true)
            .withReverseLimitEnable(true)
            .withReverseLimitAutosetPositionEnable(true)
            .withReverseLimitAutosetPositionValue(0.0))

        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(ElevatorConstants.forwardSoftLimit)
            .withForwardSoftLimitEnable(true));

    elevator.getConfigurator().apply(config, 0.2);
  }

  private Vector<N2> getOutput() {return VecBuilder.fill(position, velocity);}

  private void applyInput(Vector<N1> inputs) {
    VoltageOut config = new VoltageOut(0);
    double volts = inputs.get(0);

    if (volts < 0) {
        config.withLimitReverseMotion(reverseLimit);
    } else if (volts > 0) {
        config.withLimitForwardMotion(forwardLimit);
    }
    elevator.setControl(config.withOutput(volts));
  }

  public void setPosition(double goal) {controller.setReference(VecBuilder.fill(goal, 0.0));}
  public void setStow() {setPosition(ElevatorConstants.stow);}
  public void setProcessor() {setPosition(ElevatorConstants.processor);}
  public void setL1() {setPosition(ElevatorConstants.L1);}
  public void setL2() {setPosition(ElevatorConstants.L2);}
  public void setL3() {setPosition(ElevatorConstants.L3);}
  public void setL4() {setPosition(ElevatorConstants.L4);}
  public void setNet() {setPosition(ElevatorConstants.net);}

  public void stop() {setPosition(position);}
  public double getPosition() {return position;}

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

  public boolean atSetpoint() {return controller.isAtSetpoint();}

  @Override
  public void periodic() {
    position = cancoder.getPosition().getValueAsDouble();
    velocity = cancoder.getVelocity().getValueAsDouble();

    reverseLimit = !reverseLimiter.get();
    forwardLimit = !forwardLimiter.get();
  }
}
