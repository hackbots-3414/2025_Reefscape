package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;

public class Climber extends SubsystemBase implements AutoCloseable {
    private final Logger m_logger = LoggerFactory.getLogger(Climber.class);
    private final TalonFX m_leftClimbMotor = new TalonFX(IDConstants.climbLeft);
    private final TalonFX m_rightClimbMotor = new TalonFX(IDConstants.climbRight);

    private final DigitalInput climbedHardLimit = new DigitalInput(IDConstants.climbedHardLimit);
    private final DigitalInput climbReadyHardLimit = new DigitalInput(IDConstants.climbReadyHardLimit);

    private boolean climbedLimit = false;
    private boolean climbReadyLimit = false;

    private final Servo m_servo = new Servo(IDConstants.servo);

    private double m_voltage;
    private boolean m_voltageChanged;

    private final VoltageOut m_request = new VoltageOut(0);

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

    public void openFunnel() {
        m_servo.set(ClimberConstants.k_servoPosition);
    }

    private void setMotor(double voltage) {
        m_voltageChanged = (m_voltage != voltage);
        m_voltage = voltage;
    }

    public void setClimbUpVolts() {
        m_logger.warn("Climber motor voltages for real bot not set, set voltages in Constants.ClimberConstants.climberUpVolts");
        setMotor(ClimberConstants.climberUpVolts);
    }

    public void setDownVolts() {
        setMotor(ClimberConstants.climbDownVolts);
    }

    public void setClimbRoll() {
        m_leftClimbMotor.setControl(m_request.withOutput(ClimberConstants.climbRollVolts));
    }

    public void stopMotor() {
        m_leftClimbMotor.stopMotor();
        m_voltage = 0.0;
        m_voltageChanged = false;
    }

    @Override
    public void periodic() {
        climbedLimit = climbedHardLimit.get();
        climbReadyLimit = climbReadyHardLimit.get();

        if (m_voltageChanged) {
            if (m_voltage >= 0) {
                m_leftClimbMotor.setControl(m_request.withOutput(m_voltage).withLimitForwardMotion(climbedLimit));
            } else {
                m_leftClimbMotor.setControl(m_request.withOutput(m_voltage).withLimitReverseMotion(climbReadyLimit));
            }
            
            m_leftClimbMotor.setVoltage(m_voltage);
            m_voltageChanged = false;
        }
        SmartDashboard.putNumber("climber servo pos", m_servo.getPosition());
    }

   @Override
    public void close() throws Exception {
        m_leftClimbMotor.close();
        m_rightClimbMotor.close();
    }
}
