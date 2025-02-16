package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;

public class Climber extends SubsystemBase implements AutoCloseable {
    private final Logger m_logger = LoggerFactory.getLogger(Climber.class);
    private final TalonFX m_leftClimbMotor = new TalonFX(IDConstants.climbLeft);
    private final TalonFX m_rightClimbMotor = new TalonFX(IDConstants.climbRight);

    private double m_voltage;
    private boolean m_voltageChanged;

    public Climber() {
        configMotors();
    }

    private void configMotors() {
        m_leftClimbMotor.clearStickyFaults();
        m_rightClimbMotor.clearStickyFaults();
        m_leftClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
        m_rightClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
        m_rightClimbMotor.setControl(new Follower(IDConstants.climbLeft, ClimberConstants.rightMotorInvert));
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
        m_leftClimbMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (m_voltageChanged) {
            m_leftClimbMotor.setVoltage(m_voltage);
            m_voltageChanged = false;
        }
    }

   @Override
    public void close() throws Exception {
        m_leftClimbMotor.close();
        m_rightClimbMotor.close();
    }
}
