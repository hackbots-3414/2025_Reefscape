package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase implements AutoCloseable {
    private final Logger m_logger = LoggerFactory.getLogger(Climber.class);
    private final TalonFX leftClimbMotor = new TalonFX(ClimberConstants.leftClimberMotorID);
    private final TalonFX rightClimbMotor = new TalonFX(ClimberConstants.rightClimberMotorID);

    private double m_voltage;
    private boolean m_voltageChanged;

    public Climber() {
        configMotors();
    }

    public void configMotors() {
        leftClimbMotor.clearStickyFaults();
        rightClimbMotor.clearStickyFaults();
        leftClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
        rightClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
        rightClimbMotor.setControl(new Follower(ClimberConstants.leftClimberMotorID, ClimberConstants.rightMotorInvert));
    }

    private void setMotor(double voltage) {
        m_voltageChanged = (m_voltage != voltage);
        m_voltage = voltage;
    }

    public void setClimbUpVolts() {
        m_logger.warn("Climber motor voltages for real bot not set, set voltages in Constants.ClimberConstants.climberUpVolts");
        setMotor(ClimberConstants.climberUpVolts);
    }

    public void stopMotor() {
        leftClimbMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (m_voltageChanged) {
            leftClimbMotor.setVoltage(m_voltage);
            m_voltageChanged = false;
        }
    }

   @Override
    public void close() throws Exception {
        leftClimbMotor.close();
        rightClimbMotor.close();
    }
}