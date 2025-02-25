package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Robot;
import frc.robot.utils.RunOnChange;

public class AlgaeRollers extends SubsystemBase implements AutoCloseable {
    public enum AlgaeRollerSpeeds {STOP, INTAKE, HOLD, EJECT}

    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(AlgaeRollers.class);
    
    private final TalonFX m_algaeRoller = new TalonFX(IDConstants.algae);
    private Notifier m_notifier;
    private boolean m_hasObject;
    private RunOnChange<Double> changeVolts;

    public AlgaeRollers() {
        configMotor();
        configNotifier();
        changeVolts = new RunOnChange<>(this::writeToMotors, 0.0);
    }

    private void configMotor() {
        m_algaeRoller.clearStickyFaults();
        m_algaeRoller.getConfigurator().apply(AlgaeRollerConstants.motorConfig);
    }

    private void configNotifier() {
        m_notifier = new Notifier(this::updateObjectState);
        m_notifier.startPeriodic(AlgaeRollerConstants.k_updateObjectPeriodSeconds);
    }

    private void writeToMotors(double voltage) {
        m_algaeRoller.setVoltage(voltage);
    }

    public void setMotor(double voltage) {
        changeVolts.accept(voltage);
    }

    public boolean hasObject() {
        return m_hasObject;
    }

    private void hold() {
        setMotor(AlgaeRollerConstants.holdVoltage);
    }

    private void intake() {
        setMotor(AlgaeRollerConstants.intakeVoltage);
    }

    private void stop() {
        setMotor(0.0);
    }

    public void intakeAlgae() {
        if (hasObject()) {
            hold();
        } else {
            intake();
        }
    }

    public void smartStop() {
        if (hasObject()) {
            hold();
        } else {
            stop();
        }
    }

    public void set(AlgaeRollerSpeeds speed) {
        switch (speed) {
            case INTAKE -> intake();
            case HOLD -> hold();
            case EJECT -> eject();
            case STOP -> smartStop();
        }
    }

    private double getTorqueCurrent() {
        return m_algaeRoller.getTorqueCurrent().getValueAsDouble();
    }

    public void eject() {
        setMotor(AlgaeRollerConstants.ejectVoltage);
    }

    private void updateObjectState() {
        if (Robot.isReal()) {
            m_hasObject = getTorqueCurrent() >= AlgaeRollerConstants.torqueCurrentThreshold;
        } else {
            m_hasObject = SmartDashboard.getBoolean("Algae Holding Object", false);
        }
    }

    private void update() {}

    private void log() {
        SmartDashboard.putNumber("ALGAE VOLTS", changeVolts.getValue());
        SmartDashboard.putBoolean("Algae Holding Object", m_hasObject);
    }

    @Override
    public void periodic() {
        update();
        changeVolts.resolve();
        log();
    }

    @Override
    public void close() throws Exception {
        m_algaeRoller.close();
        m_notifier.close();
    }
}
