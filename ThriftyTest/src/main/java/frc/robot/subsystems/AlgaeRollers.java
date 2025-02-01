package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import frc.robot.Constants;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.commands.ScoreCommand;

public class AlgaeRollers extends SubsystemBase implements AutoCloseable{
    private final Logger m_logger = LoggerFactory.getLogger(ScoreCommand.class);
    private TalonFX algaeRollerMotor = new TalonFX(Constants.AlgaeRollerConstants.algaeRollerMotorID);

    public AlgaeRollers() {
        configIntakeMotor();
    }

    private void configIntakeMotor() {
        algaeRollerMotor.clearStickyFaults();

        // FIXME: Put the right inversions in for real bot
        m_logger.warn(
                "Algae roller motor inversion for real bot not set, set algae roller motor inversion for real bot");

        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        motorOutput.withNeutralMode(NeutralModeValue.Brake);
        motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        TalonFXConfigurator configurator = algaeRollerMotor.getConfigurator();

        currentConfigs.withSupplyCurrentLimit(Constants.AlgaeRollerConstants.algaeRollerCurrentLimit);
        currentConfigs.SupplyCurrentLimitEnable = true;

        configurator.apply(currentConfigs, Constants.RobotConstants.globalCanTimeout);
        configurator.apply(motorOutput, Constants.RobotConstants.globalCanTimeout);

    }

    // @Override
    // public void periodic() {
    // motorPosition = rightClimbMotor.getPosition().getValueAsDouble();
    // }

    public void setMotor(double voltage) {
        algaeRollerMotor.setVoltage(voltage);
    }

    public void intakeAlgae() {
        // TODO: Figure out if we're using stator, supply, or torque current here.
        m_logger.warn("Real bot's algaeRoller intake voltage not set, set intake voltage in Constants.AlgaeRollerConstants.intakePower");
        m_logger.warn("Real bot's algaeRoller hold voltage not set, set hold voltage in Constants.AlgaeRollerConstants.holdPower");
        m_logger.warn("Unsure if we're using stator, supply, or torque yet, please determine and set");
        if (algaeRollerMotor.getTorqueCurrent().getValueAsDouble() > Constants.AlgaeRollerConstants.currentThreshold) {
            m_logger.warn("Torque Current: "+ algaeRollerMotor.getTorqueCurrent().getValueAsDouble());
            setMotor(AlgaeRollerConstants.intakePower);
        } else if (algaeRollerMotor.getTorqueCurrent().getValueAsDouble() <= Constants.AlgaeRollerConstants.currentThreshold) {
            setMotor(AlgaeRollerConstants.holdPower);
            m_logger.warn("Torque Current: "+ algaeRollerMotor.getTorqueCurrent().getValueAsDouble());
        }
    }

    public void ejectAlgae() {
        m_logger.warn(
                "Voltage for real bot's eject  not set, set eject voltage for real bot in Constants.AlgaeRollerConstants.ejectPower");
        setMotor(AlgaeRollerConstants.ejectPower);
    }

    public void stopMotor() {
        algaeRollerMotor.setVoltage(0);
    }

    @Override
    public void close() throws Exception {
        algaeRollerMotor.close();
    }

}
