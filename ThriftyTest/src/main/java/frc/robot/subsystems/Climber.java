package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;

public class Climber extends SubsystemBase implements AutoCloseable {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(Climber.class);
    private final TalonFX m_leftClimbMotor = new TalonFX(IDConstants.climbLeft);
    private final TalonFX m_rightClimbMotor = new TalonFX(IDConstants.climbRight);
    private final CANcoder m_encoder = new CANcoder(IDConstants.climbEncoder);

    private final Servo m_servo = new Servo(IDConstants.servo);

    private double m_voltage;
    private boolean m_voltageChanged;

    private final VoltageOut m_request = new VoltageOut(0);

    public Climber() {
        configMotors();
        configEncoder();
    }

    private void configEncoder() {
        m_encoder.getConfigurator().apply(ClimberConstants.encoderConfig);
    }

    private void configMotors() {
        m_leftClimbMotor.clearStickyFaults();
        m_rightClimbMotor.clearStickyFaults();
        m_leftClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
        m_rightClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
        m_rightClimbMotor.setControl(new Follower(IDConstants.climbLeft, ClimberConstants.rightMotorInvert));
    }

    public void openFunnel() {
        m_servo.set(ClimberConstants.k_openServoPosition);
    }

    public void closeFunnel() {
        m_servo.set(ClimberConstants.k_closedServoPosition);
    }

    private void setMotor(double voltage) {
        m_voltageChanged = (m_voltage != voltage);
        m_voltage = voltage;
    }

    public void setUp() {
        setMotor(ClimberConstants.climberUpVolts);
    }

    public void setDown() {
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

    public double getEncoderValue() {
        return m_leftClimbMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        if (ClimberConstants.enable) {
            if (m_voltageChanged) {
                m_leftClimbMotor.setControl(m_request.withOutput(m_voltage));
                m_voltageChanged = false;
            }
            SmartDashboard.putNumber("climber servo pos", m_servo.getPosition());
            SmartDashboard.putNumber("climber pos", getEncoderValue());

            SmartDashboard.putBoolean("Climb Ready", climbReady());
            SmartDashboard.putBoolean("Climbed", atClimb());
        }
    }

    public boolean atClimb() {
        return getEncoderValue() <= ClimberConstants.climbPosition;
    }

    public boolean climbReady() {
        return getEncoderValue() > ClimberConstants.climbReadyTolerance;
    }

    @Override
    public void close() throws Exception {
        m_leftClimbMotor.close();
        m_rightClimbMotor.close();
    }
}
