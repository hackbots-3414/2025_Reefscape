package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeRollerConstants;

public class AlgaeRollers extends SubsystemBase implements AutoCloseable{
    
    private final Logger m_logger = LoggerFactory.getLogger(AlgaeRollers.class);
    
    private final TalonFX m_algaeRoller = new TalonFX(AlgaeRollerConstants.algaeRollerMotorID);

    private double m_voltage;
    private boolean m_voltageChanged;

    private Notifier m_notifier;

    private boolean m_hasObject;

    public AlgaeRollers() {
        configIntakeMotor();
        m_notifier = new Notifier(this::updateObjectState);
        m_notifier.startPeriodic(AlgaeRollerConstants.k_updateObjectPeriodSeconds);
    }

    private void configIntakeMotor() {
        m_algaeRoller.clearStickyFaults();

        m_logger.error("Algae roller motor inversion for real bot not set");

        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        motorOutput.withNeutralMode(NeutralModeValue.Brake);
        motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.withSupplyCurrentLimit(AlgaeRollerConstants.algaeRollerCurrentLimit);
        currentConfigs.SupplyCurrentLimitEnable = true;
        TalonFXConfigurator configurator = m_algaeRoller.getConfigurator();
        configurator.apply(currentConfigs, Constants.RobotConstants.globalCanTimeout.in(Seconds));
        configurator.apply(motorOutput, Constants.RobotConstants.globalCanTimeout.in(Seconds));
    }

    private void setMotor(double voltage) {
        if (voltage != m_voltage) {
            m_voltageChanged = true;
        }
        m_voltage = voltage;
    }

    public boolean hasObject() {
        return m_hasObject;
    }

    public void intakeAlgae() {
        if (hasObject()) {
            setMotor(AlgaeRollerConstants.holdVoltage);
            m_logger.warn("We may need current limits for having an object which are not yet implemented");
        } else {
            setMotor(AlgaeRollerConstants.intakeVoltage);
        }
    }

    private double getTorqueCurrent() {
        return m_algaeRoller.getTorqueCurrent().getValueAsDouble();
    }

    public void ejectAlgae() {
        m_logger.warn("Voltage for real bot's eject not set, set eject voltage for real bot in Constants.AlgaeRollerConstants.ejectPower");
        setMotor(AlgaeRollerConstants.ejectVoltage);
    }

    public void stopMotor() {
        setMotor(0);
    }

    private void updateObjectState() {
        m_hasObject = getTorqueCurrent() >= AlgaeRollerConstants.currentThreshold;
    }

    @Override
    public void periodic() {
        if (m_voltageChanged) {
            m_algaeRoller.setVoltage(m_voltage);
            m_voltageChanged = false;
        }

        // if (m_counter++ == AlgaeRollerConstants.k_updatePeriod - 1) {
        //     m_hasObject = getTorqueCurrent() >= AlgaeRollerConstants.currentThreshold;
        //     m_counter = 0;
        // }
    }

    @Override
    public void close() throws Exception {
        m_algaeRoller.close();
        m_notifier.close();
    }

}
