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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.utils.RunOnChange;

public class Climber extends SubsystemBase implements AutoCloseable {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(Climber.class);
    private final TalonFX m_leftClimbMotor = new TalonFX(IDConstants.climbLeft);
    private final TalonFX m_rightClimbMotor = new TalonFX(IDConstants.climbRight);

    private RunOnChange<Double> changeVolts;

    private final PIDController m_controller = new PIDController(ClimberConstants.kP, 0, 0);

    private boolean m_closedLoop = false;

    private double m_setpoint;

    private final CANrange range = new CANrange(IDConstants.canRange);
    
    private double rangeLocation = 0.0;
    private boolean climbReady = false;
    private boolean climbed = false;

    private final Servo m_servo = new Servo(IDConstants.servo);

    private final VoltageOut m_request = new VoltageOut(0);

    public Climber() {
        configMotors();
        changeVolts = new RunOnChange<>(this::writeToMotors, 0.0);
    }

    public void setClosedLoop(boolean enable) {
        m_closedLoop = enable;
        m_setpoint = m_leftClimbMotor.getPosition().getValueAsDouble();
    }

    private void configMotors() {
        m_leftClimbMotor.clearStickyFaults();
        m_rightClimbMotor.clearStickyFaults();

        m_leftClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
        m_rightClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
        
        // Required for master motor
        m_leftClimbMotor.getDutyCycle().setUpdateFrequency(0.02);
        m_leftClimbMotor.getMotorVoltage().setUpdateFrequency(0.02);
        m_leftClimbMotor.getTorqueCurrent().setUpdateFrequency(0.02);

        m_rightClimbMotor.setControl(new Follower(IDConstants.climbLeft, ClimberConstants.rightMotorInvert));
    }

    private void writeToMotors(double voltage) {
        if (voltage >= 0) {
            m_leftClimbMotor.setControl(m_request.withOutput(voltage).withLimitForwardMotion(climbed));
        } else {
            m_leftClimbMotor.setControl(m_request.withOutput(voltage).withIgnoreHardwareLimits(climbReady));
        }
    }

    public void openFunnel() {
        m_servo.set(ClimberConstants.k_openServoPosition);
    }

    public void closeFunnel() {
        m_servo.set(ClimberConstants.k_closedServoPosition);
    }

    private void setMotor(double voltage) {
        changeVolts.accept(voltage);
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
        changeVolts.instantRun(0.0);
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
        if (m_closedLoop) {
            double position = m_leftClimbMotor.getPosition().getValueAsDouble();
            double output = m_controller.calculate(position, m_setpoint);
            setMotor(output);
        }

        changeVolts.resolve();
        
        rangeLocation = range.getDistance().getValueAsDouble();

        climbReady = rangeLocation < ClimberConstants.climbReadyRangeValue;
        climbed = rangeLocation > ClimberConstants.climbedRangeValue;

        
        SmartDashboard.putNumber("climber servo pos", m_servo.getPosition());

        SmartDashboard.putBoolean("Climb Ready", climbReady);
        SmartDashboard.putBoolean("Climbed", climbed);
    }

    @Override
    public void close() throws Exception {
        m_leftClimbMotor.close();
        m_rightClimbMotor.close();
    }
}
