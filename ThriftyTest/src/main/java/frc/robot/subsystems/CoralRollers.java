// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Robot;
import frc.robot.RobotObserver;

public class CoralRollers extends SubsystemBase {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(CoralRollers.class);

    private final TalonFX m_coralLeft = new TalonFX(IDConstants.coralLeft);
    private final TalonFX m_coralRight = new TalonFX(IDConstants.coralRight);

    private final CANrange m_frontRange = new CANrange(IDConstants.coralCANrange);
    private final CANrange m_upperRange = new CANrange(IDConstants.upperCANrange);
    private final CANrange m_innerRange = new CANrange(IDConstants.innerCANrange);

    private double m_voltage;
    private boolean m_voltageChanged;
    private double m_stoppedTime;
    private boolean m_stoppedTimeChanged;

    public CoralRollers() {
        configMotors();
        configDashboard();
        configCANrange();
        RobotObserver.setPieceHeldSupplier(this::holdingPiece);
    }

    private void configMotors() {
        m_coralLeft.clearStickyFaults();
        m_coralRight.clearStickyFaults();

        m_coralLeft.getConfigurator().apply(CoralConstants.motorConfig);
        m_coralRight.getConfigurator().apply(CoralConstants.motorConfig.withMotorOutput(CoralConstants.motorConfig.MotorOutput.withInverted(CoralConstants.kInvertRight)));

        // m_coralRight.setControl(new Follower(IDConstants.coralLeft, CoralConstants.rightMotorInvert));
    }

    private void configDashboard() {
        if (Robot.isReal()) {
            // NOTHING YET
        } else {
            SmartDashboard.putBoolean("Coral Override", false);
        }
    }

    private void configCANrange() {
        m_frontRange.getConfigurator().apply(CoralConstants.frontRangeConfig);
        m_upperRange.getConfigurator().apply(CoralConstants.upperRangeConfig);
        m_innerRange.getConfigurator().apply(CoralConstants.innerRangeConfig);
    }

    public void setVoltage(double voltage) {
        m_coralLeft.setVoltage(voltage);
        m_coralRight.setVoltage(voltage);
        // m_voltageChanged = (voltage != m_voltage);
        // m_voltage = voltage;
    }

    public void setIntake() {
        setVoltage(CoralConstants.intakeVoltage);
    }

    public void setRetract() {
        setVoltage(CoralConstants.retractVoltage);
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

    public void setL2Eject() {
        m_logger.trace("Setting L2 eject");
        setVoltage(CoralConstants.l2EjectVoltage);
    }

    public void setL3Eject() {
        m_logger.trace("Setting L3 eject");
        setVoltage(CoralConstants.l3EjectVoltage);
    }

    public void setL4Eject() {
        m_logger.trace("Setting L4 eject");
        setVoltage(CoralConstants.l4EjectVoltage);
    }

    public void setSpitOut() {
        setVoltage(CoralConstants.spitOutVoltage);
    }

    public void setL1Eject() {
        m_coralLeft.setVoltage(CoralConstants.l1LeftEjectVoltage);
        m_coralRight.setVoltage(CoralConstants.l1RightEjectVoltage);
    }

    public void resetFollow() {
        m_coralRight.setControl(new Follower(IDConstants.coralLeft, CoralConstants.rightMotorInvert));
    }

    public void stop() {
        setVoltage(0);
    }

    public boolean getFrontCANrange() {
        return m_frontRange.getIsDetected().getValue();
    }

    public boolean getUpperCANrange() {
        return m_upperRange.getIsDetected().getValue();
    }

    public boolean getInnerCANrange() {
        return m_innerRange.getIsDetected().getValue();
    }

    public boolean holdingPiece() {
        if (Robot.isReal()) {
            boolean holding = getFrontCANrange() && !getUpperCANrange();
            m_logger.trace("holding: {}", holding);
            return holding;
        } else {
            boolean present = SmartDashboard.getBoolean("Coral present", false);
            SmartDashboard.putBoolean("Coral present", present);
            return present;
        }
    }

    public void fastEject() {
        setVoltage(CoralConstants.fastEjectVoltage);
    }

    public void slowScore() {
        setVoltage(CoralConstants.l1EjectVoltage);
    }

    public boolean presentPiece() {
        return getInnerCANrange() || getFrontCANrange();
    }

    public boolean intakeReady() {
        return getUpperCANrange() || presentPiece();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Inner CANrange", getInnerCANrange());
        SmartDashboard.putBoolean("Coral CANrange", getFrontCANrange());
        SmartDashboard.putBoolean("OCS", getUpperCANrange());
        SmartDashboard.putBoolean("HAS CORAL", holdingPiece());

        if (m_voltageChanged) {
            m_coralLeft.setVoltage(m_voltage);
            m_coralRight.setVoltage(m_voltage);
            m_voltageChanged = false;
        }
    }
        
}
