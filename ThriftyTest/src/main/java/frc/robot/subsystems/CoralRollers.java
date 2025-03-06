// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer.CoralLocationPresets;
import frc.robot.RobotObserver;
import frc.robot.utils.RunOnChange;

public class CoralRollers extends SubsystemBase {
    public enum CoralRollerSpeeds {
        INTAKE(CoralConstants.intakeVoltage), 
        L1(CoralConstants.l1EjectVoltage), 
        L2(CoralConstants.l2EjectVoltage),
        L3(CoralConstants.l3EjectVoltage), 
        L4(CoralConstants.l4EjectVoltage), 
        UNJAM(CoralConstants.unjamVoltage), 
        STOP(0.0);

        private double value;

        private CoralRollerSpeeds(double value) {
            this.value = value;
        }

        public static CoralRollerSpeeds fromCoralPreset(CoralLocationPresets preset) {
            CoralRollerSpeeds setpoint = CoralRollerSpeeds.STOP;
            switch (preset) {
                case L1 -> setpoint = CoralRollerSpeeds.L1;
                case L2 -> setpoint = CoralRollerSpeeds.L2;
                case L3 -> setpoint = CoralRollerSpeeds.L3;
                case L4 -> setpoint = CoralRollerSpeeds.L4;
                case LEFT_INTAKE -> setpoint = CoralRollerSpeeds.INTAKE;
                case RIGHT_INTAKE -> setpoint = CoralRollerSpeeds.INTAKE;
            }
            return setpoint;
        }
    }


    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(CoralRollers.class);

    private final TalonFX m_coralLeft = new TalonFX(IDConstants.coralLeft);
    private final TalonFX m_coralRight = new TalonFX(IDConstants.coralRight);

    private final AnalogInput m_frontIR = new AnalogInput(IDConstants.frontIR);
    private final AnalogInput m_backIR = new AnalogInput(IDConstants.rearIR);

    private boolean m_frontSensorValue = false;
    private boolean m_backSensorValue = false;

    private RunOnChange<Double> changeVolts;

    public CoralRollers() {
        configMotors();
        configDashboard();
        changeVolts = new RunOnChange<>(this::writeToMotors, 0.0);
        RobotObserver.setPieceHeldSupplier(this::holdingPiece);
        
        // Disables status signals communicated that don't need to be
        ParentDevice.optimizeBusUtilizationForAll(m_coralLeft, m_coralRight);
    }

    private void configMotors() {
        m_coralLeft.clearStickyFaults();
        m_coralRight.clearStickyFaults();

        m_coralLeft.getConfigurator().apply(CoralConstants.motorConfig);
        m_coralRight.getConfigurator().apply(CoralConstants.motorConfig);
        
        // Required for master motor
        m_coralLeft.getDutyCycle().setUpdateFrequency(0.02);
        m_coralLeft.getMotorVoltage().setUpdateFrequency(0.02);
        m_coralLeft.getTorqueCurrent().setUpdateFrequency(0.02);

        m_coralRight.setControl(new Follower(IDConstants.coralLeft, CoralConstants.rightMotorInvert));
    }

    private void configDashboard() {
        if (Robot.isReal()) {
            // NOTHING YET
        } else {
            SmartDashboard.putBoolean("Coral Override", false);
        }
    }

    private void writeToMotors(double voltage) {
        m_coralLeft.setVoltage(voltage);
    }

    public void setVoltage(double voltage) {
        changeVolts.accept(voltage);
    }

    public void set(CoralRollerSpeeds speed) {
        setVoltage(speed.value);
    }

    public void unjam() {
        setVoltage(CoralConstants.unjamVoltage);
    }

    public void stop() {
        changeVolts.run(0.0);
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

    private void update() {
        if (Robot.isReal()) {
            m_frontSensorValue = m_frontIR.getVoltage() > CoralConstants.frontIRThreshold;
            m_backSensorValue = m_backIR.getVoltage() > CoralConstants.frontIRThreshold;
        } else {
            m_frontSensorValue = SmartDashboard.getBoolean("Coral Override", false);
            m_backSensorValue = SmartDashboard.getBoolean("Coral Override", false);
        }
        
        changeVolts.resolveIfChange();
    }

    private void log() {
        SmartDashboard.putBoolean("Front IR Triggered", m_frontSensorValue);
        SmartDashboard.putBoolean("Rear IR Triggered", m_backSensorValue);
        SmartDashboard.putBoolean("Holding Coral", holdingPiece());
        SmartDashboard.putNumber("Coral Roller Volts", changeVolts.getValue());
    }

    @Override
    public void periodic() {
        update();
        log();
    }
}
