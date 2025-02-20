// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Robot;

public class CoralRollers extends SubsystemBase {
    private final TalonFX m_coralLeft = new TalonFX(IDConstants.coralLeft);
    private final TalonFX m_coralRight = new TalonFX(IDConstants.coralRight);

    private final AnalogInput m_frontIR = new AnalogInput(IDConstants.frontIR); // tolerance 1
    private final AnalogInput m_backIR = new AnalogInput(IDConstants.rearIR);

    private boolean m_frontSensorValue = false;
    private boolean m_backSensorValue = false;

    private double m_voltage;
    private boolean m_voltageChanged;
    private double m_stoppedTime;
    private boolean m_stoppedTimeChanged;

    public CoralRollers() {
        configMotors();
    }

    private void configMotors() {
        m_coralLeft.clearStickyFaults();
        m_coralRight.clearStickyFaults();

        m_coralLeft.getConfigurator().apply(CoralConstants.motorConfig);
        m_coralRight.getConfigurator().apply(CoralConstants.motorConfig);

        m_coralRight.setControl(new Follower(IDConstants.coralLeft, CoralConstants.rightMotorInvert));
    }

    public void setVoltage(double voltage) {
        m_voltageChanged = (voltage != m_voltage);
        m_voltage = voltage;
    }

    public void setIntake() {
        if (holdingPiece()) {
            stop();
        } else {
            setVoltage(CoralConstants.intakeVoltage);
        }
    }

    public void timeoutIntake() {
        // a whole lotta logic that essentially allows u to stop the motor in default command
        // when you leave the yay zone after intakeTimeout seconds automatically (max process time) or when coral detected
        if (!m_stoppedTimeChanged) {
            m_stoppedTimeChanged = true;
            m_stoppedTime = Utils.getCurrentTimeSeconds();
        }

        if (holdingPiece()) {
            stop();
            return;
        }

        if (presentPiece()) {
            setIntake();
            return;
        }

        double elapsed = Utils.getCurrentTimeSeconds() - m_stoppedTime;

        if (elapsed > CoralConstants.intakeTimeout) {
            stop();
            resetTimeout();
            return;
        }

        if (holdingPiece()) {
            stop();
        }
    }

    public void resetTimeout() {
        m_stoppedTime = -1;
        m_stoppedTimeChanged = false;
    }

    public void setEject() {
        setVoltage(CoralConstants.ejectVoltage);
    }

    public void stop() {
        System.out.println("stopping");
        setVoltage(0);
    }

    public boolean getFrontIR() {
        return m_frontSensorValue;
    }

    public boolean getBackIR() {
        return m_backSensorValue;
    }

    public boolean holdingPiece() {
        return getFrontIR() && getBackIR();
    }

    public boolean presentPiece() {
        return getFrontIR() || getBackIR();
    }

    @Override
    public void periodic() {
        if (Robot.isReal()) {
            m_frontSensorValue = m_frontIR.getVoltage() > CoralConstants.frontIRThreshold;
            m_backSensorValue = m_backIR.getVoltage() > CoralConstants.frontIRThreshold;
        } else {
            m_frontSensorValue = SmartDashboard.getBoolean("Front IR", false);
            m_backSensorValue = SmartDashboard.getBoolean("Back IR", false);
        }

        SmartDashboard.putBoolean("Front IR Triggered", m_frontSensorValue);
        SmartDashboard.putBoolean("Rear IR Triggered", m_backSensorValue);

        SmartDashboard.putNumber("Front IR", m_frontIR.getVoltage());
        SmartDashboard.putNumber("Back IR", m_backIR.getVoltage());

        SmartDashboard.putBoolean("HAS CORAL", holdingPiece());

        if (m_voltageChanged) {
            m_coralLeft.setVoltage(m_voltage);
            m_voltageChanged = false;
            SmartDashboard.putNumber("* CORAL VOLTS", m_voltage);
        }
    }
}
