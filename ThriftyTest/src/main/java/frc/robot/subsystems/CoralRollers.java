// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Robot;
import frc.robot.utils.RunOnChange;

public class CoralRollers extends SubsystemBase {
    public enum CoralRollerSpeeds {INTAKE, L1, L2, L3, L4, UNJAM, L1_SEPERATE, STOP}

    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(CoralRollers.class);

    private final TalonFX m_coralLeft = new TalonFX(IDConstants.coralLeft);
    private final TalonFX m_coralRight = new TalonFX(IDConstants.coralRight);

    private final AnalogInput m_frontIR = new AnalogInput(IDConstants.frontIR); // tolerance 1
    private final AnalogInput m_backIR = new AnalogInput(IDConstants.rearIR);

    private boolean m_frontSensorValue = false;
    private boolean m_backSensorValue = false;

    private RunOnChange<Double> changeVolts;
    private RunOnChange<Pair<Double, Double>> changeIndividualVolts;
    
    private double m_stoppedTime;
    private boolean m_stoppedTimeChanged;

    public CoralRollers() {
        configMotors();
        configDashboard();
        changeVolts = new RunOnChange<>(this::writeToMotors, 0.0);
        changeIndividualVolts = new RunOnChange<Pair<Double, Double>>(this::writeIndividual, Pair.of(0.0, 0.0));
    }

    private void configMotors() {
        m_coralLeft.clearStickyFaults();
        m_coralRight.clearStickyFaults();

        m_coralLeft.getConfigurator().apply(CoralConstants.motorConfig);
        m_coralRight.getConfigurator().apply(CoralConstants.motorConfig);

        m_coralRight.setControl(new Follower(IDConstants.coralLeft, CoralConstants.rightMotorInvert));
    }

    private void configDashboard() {
        if (Robot.isReal()) {
            // NOTHING YET
        } else {
            SmartDashboard.putBoolean("Front IR", false);
            SmartDashboard.putBoolean("Back IR", false);
        }
    }

    private void writeToMotors(double voltage) {
        m_coralLeft.setVoltage(voltage);
    }

    private void writeIndividual(Pair<Double, Double> individualSpeeds) {
        m_coralLeft.setVoltage(CoralConstants.l1LeftEjectVoltage);
        m_coralRight.setVoltage(CoralConstants.l1RightEjectVoltage);
    }

    public void setVoltage(double voltage) {
        changeVolts.accept(voltage);
    }

    public void setIndividualEject() {
        changeIndividualVolts.accept(new Pair<Double,Double>(CoralConstants.l1LeftEjectVoltage, CoralConstants.l1RightEjectVoltage));
    }

    public void intake() {
        setVoltage(CoralConstants.intakeVoltage);
    }

    public void timeoutIntake() {
        if (!m_stoppedTimeChanged) {
            m_stoppedTimeChanged = true;
            m_stoppedTime = Utils.getCurrentTimeSeconds();
        }

        if (holdingPiece()) {
            stop();
            return;
        }

        if (presentPiece()) {
            set(CoralRollerSpeeds.INTAKE);
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

    public void set(CoralRollerSpeeds speed) {
        switch (speed) {
            case INTAKE -> setVoltage(CoralConstants.intakeVoltage);
            case L1 -> setVoltage(CoralConstants.l1EjectVoltage);
            case L2 -> setVoltage(CoralConstants.l2EjectVoltage);
            case L3 -> setVoltage(CoralConstants.l3EjectVoltage);
            case L4 -> setVoltage(CoralConstants.l4EjectVoltage);
            case UNJAM -> setVoltage(CoralConstants.unjamVoltage);
            case L1_SEPERATE -> setIndividualEject();
            case STOP -> setVoltage(0.0);
        }
    }

    public void resetFollow() {
        m_coralRight.setControl(new Follower(IDConstants.coralLeft, CoralConstants.rightMotorInvert));
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
            m_frontSensorValue = SmartDashboard.getBoolean("Front IR", false);
            m_backSensorValue = SmartDashboard.getBoolean("Back IR", false);
        }
    }

    private void log() {
        SmartDashboard.putBoolean("Front IR Triggered", m_frontSensorValue);
        SmartDashboard.putBoolean("Rear IR Triggered", m_backSensorValue);
        SmartDashboard.putBoolean("HAS CORAL", holdingPiece());
        SmartDashboard.putNumber("CORAL VOLTAGE", changeVolts.getValue());
    }

    @Override
    public void periodic() {
        update();
        changeVolts.resolveIfChange();
        changeIndividualVolts.resolveIfChange();
        log();
    }
}
