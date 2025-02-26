// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Robot;
import frc.robot.utils.RunOnChange;

public class Pivot extends SubsystemBase {
    public enum PivotSetpoints {GROUND, STOW, PROCESSOR, REEF_PICKUP, REEF_EXTRACT, NET}

    private final TalonFX m_pivot = new TalonFX(IDConstants.pivot);
    private final CANcoder m_cancoder = new CANcoder(IDConstants.pivotEncoder);

    private double m_position;
    private double m_velocity;

    private double m_reference;

    private SingleJointedArmSim m_armSim;
    private final DCMotor m_gearbox = DCMotor.getKrakenX60(1); // 2 motors (left and right)

    private Mechanism2d m_mechVisual;
    private MechanismRoot2d m_mechRoot;
    private MechanismLigament2d m_armLigament;

    private RunOnChange<Double> changeVolts;
    private RunOnChange<Double> changeSetpoint;

    public Pivot() {
        configSim();
        configEncoder();
        configMotor();
        changeVolts = new RunOnChange<>(this::writeToMotors, 0.0);
        changeSetpoint = new RunOnChange<>(this::writePositionToMotors, PivotConstants.stow);
    }

    private void configEncoder() {
        m_cancoder.clearStickyFaults();
        m_cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);
        m_cancoder.getConfigurator().apply(PivotConstants.encoderConfig, 0.2);
    }

    private void configMotor() {
        m_pivot.getConfigurator().apply(PivotConstants.motorConfig, 0.2);
    }

    private void configSim() {
        m_armSim = new SingleJointedArmSim(
                PivotConstants.stateSpacePlant,
                m_gearbox,
                PivotConstants.gearRatio,
                PivotConstants.armLength,
                PivotConstants.radiansAtZero,
                PivotConstants.radiansAtMax,
                true, // Add noise for realism
                PivotConstants.stow // Starting angle
        );

        m_mechVisual = new Mechanism2d(1.0, 1.0); // Width/height in meters
        m_mechRoot = m_mechVisual.getRoot("ArmRoot", 0.5, 0.0); // Center at (0.5, 0)
        m_armLigament = m_mechRoot
                .append(new MechanismLigament2d("Arm", PivotConstants.armLength, Math.toDegrees(m_position)));
        SmartDashboard.putData("Pivot Arm Visualization", m_mechVisual);
    }

    private void writeToMotors(double voltage) {
        m_pivot.setVoltage(voltage);
    }

    MotionMagicVoltage control = new MotionMagicVoltage(0);

    private void writePositionToMotors(double setpoint) {
        m_pivot.setControl(control.withPosition(setpoint));
    }

    public void setPosition(double goal) {
        changeSetpoint.accept(goal);
    }

    public void setVoltage(double voltage) {
        changeVolts.accept(voltage);
    }

    public void set(PivotSetpoints setpoint) {
        switch (setpoint) {
            case GROUND -> setPosition(PivotConstants.groundPickup);
            case STOW -> setPosition(PivotConstants.stow);
            case PROCESSOR -> setPosition(PivotConstants.processor);
            case REEF_PICKUP -> setPosition(PivotConstants.reefPickup);
            case REEF_EXTRACT -> setPosition(PivotConstants.reefExtract);
            case NET-> setPosition(PivotConstants.net);
        }
    }

    public void stow() {
        setPosition(PivotConstants.stow);
    }
    
    public void stop() {
        setPosition(m_position);
    }

    public double getPosition() {
        return m_position;
    }

    public double getReference() {
        return m_reference;
    }

    public double getVelocity() {
        return m_velocity;
    }
    
    public boolean atSetpoint() {
        return Math.abs(getReference() - getPosition()) < PivotConstants.tolerance;
    }
    
    private double getPositionUncached() {
        if (Robot.isReal()) {
            return m_pivot.getPosition().getValueAsDouble();
        } else {
            return m_armSim.getAngleRads();
        }
    }

    private double getVelocityUncached() {
        if (Robot.isReal()) {
            return m_pivot.getVelocity().getValueAsDouble();
        } else {
            return m_armSim.getVelocityRadPerSec();
        }
    }

    private void update() {
        m_position = getPositionUncached();
        m_velocity = getVelocityUncached();

        changeVolts.resolveIfChange();
        changeSetpoint.resolveIfChange();
    }

    private void log() {
        SmartDashboard.putNumber("REFERENCE FOR PIVOT", getReference());
        SmartDashboard.putNumber("POSITION FOR PIVOT", getPosition());
        SmartDashboard.putBoolean("PIVOT AT POSITION", atSetpoint());
    }

    @Override
    public void periodic() {
        update();
        log();
    }

    @Override
    public void simulationPeriodic() {
        m_armLigament.setAngle(Math.toDegrees(m_position));
        // Update the simulation with the motor voltage
        double appliedVolts = m_pivot.get() * RobotController.getBatteryVoltage();
        m_armSim.setInput(appliedVolts);
        m_armSim.update(SimConstants.k_simPeriodic);

        // Update the simulated encoder values
        m_cancoder.getSimState().setRawPosition(m_position / (2 * Math.PI)); // Convert radians to rotations
        m_cancoder.getSimState().setVelocity(m_velocity / (2 * Math.PI)); // Convert rad/s to RPM

        // Simulate battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    }
}
