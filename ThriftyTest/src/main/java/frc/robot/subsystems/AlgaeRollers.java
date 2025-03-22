package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Robot;
import frc.robot.RobotObserver;

public class AlgaeRollers extends SubsystemBase implements AutoCloseable {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(AlgaeRollers.class);
    
    private final TalonFX m_algaeRoller = new TalonFX(IDConstants.algae);

    private double m_voltage;
    private boolean m_voltageChanged;

    private boolean m_hasObject;

    private MedianFilter m_filter = new MedianFilter(10);

    public AlgaeRollers() {
        configIntakeMotor();
        RobotObserver.setAlgaePieceHeldSupplier(this::hasObject);

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
        double measurement = m_algaeRoller.getTorqueCurrent().getValueAsDouble();
        return m_filter.calculate(measurement);
    }

    public void ejectAlgae() {
        setMotor(AlgaeRollerConstants.ejectVoltage);
    }

    public void processorEjectAlgae() {
        setMotor(AlgaeRollerConstants.processorEjectVoltage);
    }

    public void stopMotor() {
        setMotor(0);
    }

    private void updateObjectState() {
        if (AlgaeRollerConstants.enable) {
            if (Robot.isReal()) {
                m_hasObject = getTorqueCurrent() >= AlgaeRollerConstants.torqueCurrentThreshold;
            } else {
                m_hasObject = SmartDashboard.getBoolean("Algae Holding Object", false);
            }
    
            SmartDashboard.putBoolean("Algae Holding Object", m_hasObject);
        }
    }

    @Override
    public void periodic() {
        updateObjectState();
        if (AlgaeRollerConstants.enable) {
            if (m_voltageChanged) {
                m_algaeRoller.setVoltage(m_voltage);
                m_voltageChanged = false;
                SmartDashboard.putNumber("ALGAE VOLTS", m_voltage);
            }
        }
        SmartDashboard.putNumber("algae temp", m_algaeRoller.getDeviceTemp().getValueAsDouble());
    }

    @Override
    public void close() throws Exception {
        m_algaeRoller.close();
    }

}
