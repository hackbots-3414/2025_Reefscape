// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralRollers extends SubsystemBase {
  private TalonFX coralLeft = new TalonFX(CoralConstants.k_leftMotorID);
  private TalonFX coralRight = new TalonFX(CoralConstants.k_rightMotorID);

  private DigitalInput frontSensor = new DigitalInput(CoralConstants.k_frontSensorPort);
  private DigitalInput backSensor = new DigitalInput(CoralConstants.k_backSensorPort);

  private boolean frontSensorValue = false;
  private boolean backSensorValue = false;

  private double m_voltage;
  private boolean m_voltageChanged;
  
  public CoralRollers() {
    configMotors();
  }

  private void configMotors() {
    coralLeft.clearStickyFaults();
    coralRight.clearStickyFaults();

    coralLeft.getConfigurator().apply(CoralConstants.motorConfig);
    coralRight.getConfigurator().apply(CoralConstants.motorConfig);

    coralRight.setControl(new Follower(CoralConstants.k_leftMotorID, CoralConstants.rightMotorInvert));
  }

  public void setVoltage(double voltage) {
    m_voltageChanged = (voltage != m_voltage);
    m_voltage = voltage;
  }

  public void setIntake() {
    setVoltage(CoralConstants.intakeVoltage);
  }

  public void setEject() {
    setVoltage(CoralConstants.ejectVoltage);
  }

  public void stop() {
    setVoltage(0);
  }

  public boolean getFrontIR() {
    return frontSensorValue;
  }

  public boolean getBackIR() {
    return backSensorValue;
  }

  public boolean holdingPiece() {
    return getFrontIR() || getBackIR();
  }

  @Override
  public void periodic() {
    frontSensorValue = frontSensor.get();
    backSensorValue = backSensor.get();

    if (m_voltageChanged) {
      coralLeft.setVoltage(m_voltage);
    }
  }

  //TODO Set current limits

  //TODO force neutral mode to coast
}
