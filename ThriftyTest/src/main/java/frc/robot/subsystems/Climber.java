package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanRangeConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;

public class Climber extends SubsystemBase implements AutoCloseable {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(Climber.class);
    private final TalonFX m_leftClimbMotor = new TalonFX(IDConstants.climbLeft);
    private final TalonFX m_rightClimbMotor = new TalonFX(IDConstants.climbRight);

    private final PIDController m_controller = new PIDController(ClimberConstants.kP, 0, 0);

    private boolean m_closedLoop = false;

    private double m_setpoint;

    private final CANrange range = new CANrange(IDConstants.canRange);
    
    private double rangeLocation = 0.0;
    private boolean climbReady = false;
    private boolean climbed = false;

    private final Servo m_servo = new Servo(IDConstants.servo);

    private double m_voltage;
    private boolean m_voltageChanged;

    private final VoltageOut m_request = new VoltageOut(0);

    public Climber() {
        configMotors();
        configCanRange();
    }

    private void configCanRange() {
        range.getConfigurator().apply(CanRangeConstants.k_canRangeConfig);
    }

    public void setClosedLoop(boolean enable) {
        m_closedLoop = enable;
        m_setpoint = getEncoderValue();
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
        m_closedLoop = false;
    }

    public void setClimbUpVolts() {
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

    public boolean climbed() {
        return climbed;
    }

    public boolean ready() {
        return climbReady;
    }

    public double getEncoderValue() {
        return m_leftClimbMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        if (ClimberConstants.enable) {
            rangeLocation = range.getDistance().getValueAsDouble();

        climbReady = rangeLocation < ClimberConstants.climbReadyRangeValue;
        climbed = rangeLocation > ClimberConstants.climbedRangeValue;

        if (m_voltageChanged) {
            if (m_voltage >= 0) {
                m_leftClimbMotor.setControl(m_request.withOutput(m_voltage).withLimitForwardMotion(climbed));
            } else {
                m_leftClimbMotor.setControl(m_request.withOutput(m_voltage).withIgnoreHardwareLimits(climbReady));
            }            
            m_voltageChanged = false;
        }
        SmartDashboard.putNumber("climber servo pos", m_servo.getPosition());

        SmartDashboard.putBoolean("Climb Ready", climbReady);
        SmartDashboard.putBoolean("Climbed", climbed);

        if (m_closedLoop) {
            double position = getEncoderValue();
            double output = m_controller.calculate(position, m_setpoint);
            setMotor(output);
        }
    }

   @Override
    public void close() throws Exception {
        m_leftClimbMotor.close();
        m_rightClimbMotor.close();
    }
}
