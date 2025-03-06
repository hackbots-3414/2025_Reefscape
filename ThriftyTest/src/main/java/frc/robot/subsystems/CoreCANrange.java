// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.opencv.core.Core;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.hardware.core.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanRangeConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ButtonBindingConstants.ButtonBoardAlternate;

import  com.ctre.phoenix6.hardware.CANrange;


/** Add your docs here. */
public class CoreCANrange extends SubsystemBase {
   private static double matchTime = 0;
   private boolean inAuton = false;
    private boolean inTeleop = false;
    private int value = 0; 
 // Constants used in CANrange construction
 
 // Construct the CANrange
 public static CANrange CANrange = new CANrange(IDConstants.CANrange,  CanRangeConstants.kCANrangeCANbus);
 
 // Configure the CANrange for basic use
 CANrangeConfiguration configs = new CANrangeConfiguration();
 
 // Write these configs to the CANrange
 
 CANrangeConfigurator configurator = CANrange.getConfigurator();

 // Get Distance
  public static StatusSignal<Distance> distance = CANrange.getDistance();
 
 
    
    public CoreCANrange () {
        CANrange.getConfigurator().apply(configs);
     }

    @Override
    public void periodic() {
        matchTime = DriverStation.getMatchTime();
        inAuton = DriverStation.isAutonomousEnabled();
        inTeleop = DriverStation.isTeleopEnabled();


        // if (inTeleop) {
        //     if () {

        //     }
        // }




    }

}