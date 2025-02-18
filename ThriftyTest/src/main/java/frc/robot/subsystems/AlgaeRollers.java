package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.Constants.IDConstants;

public class AlgaeRollers extends SubsystemBase implements AutoCloseable {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(AlgaeRollers.class);
    
    private final TalonFX m_algaeRoller = new TalonFX(IDConstants.algaeMotor);

    private double m_voltage;
    private boolean m_voltageChanged;

    private final Notifier m_notifier;

    private boolean m_hasObject;

    public AlgaeRollers() {
        configIntakeMotor();
        m_notifier = new Notifier(this::updateObjectState);
        m_notifier.startPeriodic(AlgaeRollerConstants.k_updateObjectPeriodSeconds);
    }

    private void configIntakeMotor() {
        m_algaeRoller.clearStickyFaults();
        m_algaeRoller.getConfigurator().apply(AlgaeRollerConstants.motorConfig);
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
        } else {
            setMotor(AlgaeRollerConstants.intakeVoltage);
        }
    }

    public void smartStop() {
        if (hasObject()) {
            setMotor(AlgaeRollerConstants.holdVoltage);
        } else {
            stopMotor();
        }
    }

    private double getTorqueCurrent() {
        return m_algaeRoller.getTorqueCurrent().getValueAsDouble();
    }

    public void ejectAlgae() {
        setMotor(AlgaeRollerConstants.ejectVoltage);
    }

    public void stopMotor() {
        setMotor(0);
    }

    private void updateObjectState() {
        if (Robot.isReal()) {
            m_hasObject = getTorqueCurrent() >= AlgaeRollerConstants.torqueCurrentThreshold;
        } else {
            m_hasObject = SmartDashboard.getBoolean("Algae Holding Object", false);
        }

        SmartDashboard.putBoolean("Algae Holding Object", m_hasObject);
    }

    @Override
    public void periodic() {
        if (m_voltageChanged) {
            m_algaeRoller.setVoltage(m_voltage);
            m_voltageChanged = false;
            SmartDashboard.putNumber("ALGAE VOLTS", m_voltage);
        }
    }

    @Override
    public void close() throws Exception {
        m_algaeRoller.close();
        m_notifier.close();
    }

}
