// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
  private TalonFX coralLeft = new TalonFX(CoralConstants.left_motorID) ;
  private TalonFX coralRight = new TalonFX(CoralConstants.right_motorID) ;

  private DigitalInput frontSensor = new DigitalInput(CoralConstants.k_frontSensorPort) ;
  private DigitalInput backSensor = new DigitalInput(CoralConstants.k_backSensorPort) ;

  private boolean frontSensorValue = false;
  private boolean backSensorValue = false;
  
  private MedianFilter m_Filter = new MedianFilter(5);

  public Coral() {
    configMotors();
  }

  private void configMotors() {
    coralLeft.clearStickyFaults();
    coralRight.clearStickyFaults();

    coralLeft.getConfigurator().apply(CoralConstants.motorConfig);
    coralRight.getConfigurator().apply(CoralConstants.motorConfig);

    coralRight.setControl(new Follower(CoralConstants.left_motorID, CoralConstants.rightMotorInvert));
  }

  public void setVoltage(double voltage) {
    coralLeft.setVoltage(voltage);
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
  }

  //TODO Set current limits

  //TODO Add method that uses current sensing for coral

  //TODO force neutral mode to coast
}
