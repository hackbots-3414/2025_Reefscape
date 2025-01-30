package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase implements AutoCloseable {

    private TalonFX leftClimbMotor = new TalonFX(ClimberConstants.leftClimberMotorID);
    private TalonFX rightClimbMotor = new TalonFX(ClimberConstants.rightClimberMotorID);
    private double motorPosition;

    public Climber() {
        configMotors();
    }

    public void configMotors() {
        leftClimbMotor.clearStickyFaults();
        rightClimbMotor.clearStickyFaults();

        //FIXME: Put the right inversions in for real bot
        Constants.ClimberConstants.currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftClimbMotor.getConfigurator().apply(Constants.ClimberConstants.currentConfigs);
        Constants.ClimberConstants.currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightClimbMotor.getConfigurator().apply(Constants.ClimberConstants.currentConfigs);

        TalonFXConfiguration configuration = new TalonFXConfiguration();

        rightClimbMotor.getConfigurator().apply(configuration, 0.2);

        rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);
        leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);

        leftClimbMotor.setControl(new Follower(rightClimbMotor.getDeviceID(), false));
    }

    @Override
    public void periodic() {
        motorPosition = rightClimbMotor.getPosition().getValueAsDouble();
    }

    public void setMotor(double voltage) {
        rightClimbMotor.setVoltage(voltage);
        leftClimbMotor.setVoltage(voltage);
    }

    public void setClimbUpVolts() {
        setMotor(ClimberConstants.climberUpVolts);
    }

    public void setClimbDownVolts() {
        setMotor(ClimberConstants.climberDownVolts);
    }

    public void stopMotor() {
        rightClimbMotor.setVoltage(0);
        leftClimbMotor.setVoltage(0);
    }

    public double getMotorPos() {
        return motorPosition;
    }


    @Override
    public void close() throws Exception {
        leftClimbMotor.close();
        rightClimbMotor.close();
    }
}