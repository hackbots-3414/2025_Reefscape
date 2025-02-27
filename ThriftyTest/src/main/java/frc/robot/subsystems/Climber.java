package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private final Servo m_servo = new Servo(IDConstants.servo);

    public Climber() {
        configMotors();
        changeVolts = new RunOnChange<>(this::writeToMotors, 0.0);
    }

    private void configMotors() {
        m_leftClimbMotor.clearStickyFaults();
        m_rightClimbMotor.clearStickyFaults();
        m_leftClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
        m_rightClimbMotor.getConfigurator().apply(ClimberConstants.motorConfig);
        m_rightClimbMotor.setControl(new Follower(IDConstants.climbLeft, ClimberConstants.rightMotorInvert));
    }

    private void writeToMotors(double voltage) {
        m_leftClimbMotor.setVoltage(voltage);
    }

    public void openFunnel() {
        m_servo.set(ClimberConstants.k_servoPosition);
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

    public void stopMotor() {
        changeVolts.instantRun(0.0);
    }

    @Override
    public void periodic() {
        changeVolts.resolve();
        SmartDashboard.putNumber("climber servo pos", m_servo.getPosition());
    }

    @Override
    public void close() throws Exception {
        m_leftClimbMotor.close();
        m_rightClimbMotor.close();
    }
}
