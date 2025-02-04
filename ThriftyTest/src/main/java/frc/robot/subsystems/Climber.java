package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

        m_logger.warn("Climber motor inversions for real bot not set, set climber inversions for real bot");

        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        motorOutput.withNeutralMode(NeutralModeValue.Brake);
        motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.withSupplyCurrentLimit(Constants.ClimberConstants.climberCurrentLimit);
        currentConfigs.SupplyCurrentLimitEnable = true;
        TalonFXConfigurator configurator = rightClimbMotor.getConfigurator();
        configurator.apply(currentConfigs, Constants.RobotConstants.globalCanTimeout);
        configurator.apply(motorOutput, Constants.RobotConstants.globalCanTimeout);
        configurator = leftClimbMotor.getConfigurator();
        configurator.apply(currentConfigs, Constants.RobotConstants.globalCanTimeout);
        configurator.apply(motorOutput, Constants.RobotConstants.globalCanTimeout);

        leftClimbMotor.setControl(new Follower(rightClimbMotor.getDeviceID(), true));
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
        rightClimbMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (m_voltageChanged) {
            rightClimbMotor.setVoltage(m_voltage);
        }
    }

    @Override
    public void close() throws Exception {
        leftClimbMotor.close();
        rightClimbMotor.close();
    }
}