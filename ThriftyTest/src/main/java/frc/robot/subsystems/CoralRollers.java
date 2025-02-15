// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.IDConstants;

public class CoralRollers extends SubsystemBase {
    private final TalonFX coralLeft = new TalonFX(IDConstants.coralLeft);
    private final TalonFX coralRight = new TalonFX(IDConstants.coralRight);

    private final DigitalInput frontSensor = new DigitalInput(IDConstants.coralFrontIR);
    private final DigitalInput backSensor = new DigitalInput(IDConstants.coralRearIR);

    private boolean frontSensorValue = false;
    private boolean backSensorValue = false;

    private double m_voltage;
    private boolean m_voltageChanged;
    private double m_stoppedTime;
    private boolean m_stoppedTimeChanged;

    public CoralRollers() {
        configMotors();
    }

    private void configMotors() {
        coralLeft.clearStickyFaults();
        coralRight.clearStickyFaults();

        coralLeft.getConfigurator().apply(CoralConstants.motorConfig);
        coralRight.getConfigurator().apply(CoralConstants.motorConfig);

        coralRight.setControl(new Follower(IDConstants.coralLeft, CoralConstants.rightMotorInvert));
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
        setVoltage(0);
    }

    public boolean getFrontIR() {
        return frontSensorValue;
    }

    public boolean getBackIR() {
        return backSensorValue;
    }

    public boolean holdingPiece() {
        return getFrontIR() && getBackIR();
    }

    public boolean presentPiece() {
        return getFrontIR() || getBackIR();
    }

    @Override
    public void periodic() {
        frontSensorValue = frontSensor.get();
        backSensorValue = backSensor.get();

        if (m_voltageChanged) {
            coralLeft.setVoltage(m_voltage);
            m_voltageChanged = false;
            SmartDashboard.putNumber("* CORAL VOLTS", m_voltage);
        }
    }
}
