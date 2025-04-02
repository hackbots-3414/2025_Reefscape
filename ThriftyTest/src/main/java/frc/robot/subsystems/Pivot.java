// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.RobotObserver;

public class Pivot extends SubsystemBase {
    private final TalonFX m_pivot = new TalonFX(IDConstants.pivot);

    private double m_position;
    private double m_velocity;

    private double m_reference;

    private SingleJointedArmSim m_armSim;
    private final DCMotor m_gearbox = DCMotor.getKrakenX60(1); // 2 motors (left and right)

    private Mechanism2d m_mechVisual;
    private MechanismRoot2d m_mechRoot;
    private MechanismLigament2d m_armLigament;

    private double m_speed;
    private boolean m_speedChanged;

    public Pivot() {
        configSim();
        configMotor();
    }

    private void configMotor() {
        m_pivot.getConfigurator().apply(PivotConstants.motorConfig, 0.2);
        m_pivot.setPosition(PivotConstants.rotorOffset);
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

    MotionMagicVoltage control = new MotionMagicVoltage(0);

    public void setPosition(double goal) {
        if (RobotObserver.getAlgaePieceHeld()) {
            m_pivot.setControl(control.withPosition(goal).withSlot(1));
        } else {
            m_pivot.setControl(control.withPosition(goal).withSlot(0));
        }
        m_reference = goal;
    }

    public void setSpeed(double speed) {
        m_speedChanged = (speed != m_speed);
        m_speed = speed;
    }

    public void setStow() {
        setPosition(PivotConstants.stow);
    }

    public void setProcessor() {
        setPosition(PivotConstants.processor);
    }

    public void setNet() {
        setPosition(PivotConstants.net);
    }

    public void setGroundPickup() {
        setPosition(PivotConstants.groundPickup);
    }

    public void setReefPickup() {
        setPosition(PivotConstants.reefPickup);
    }

    public void setReefExtract() {
        setPosition(PivotConstants.reefExtract);
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

    @Override
    public void periodic() {
        m_armLigament.setAngle(Math.toDegrees(m_position));

        m_position = getPositionUncached();
        m_velocity = getVelocityUncached();

        if (m_speedChanged) {
            m_pivot.setControl(new DutyCycleOut(m_speed));
            m_speedChanged = false;
        }

        SmartDashboard.putBoolean("PIVOT AT POSITION", atSetpoint());
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation with the motor voltage
        double appliedVolts = m_pivot.get() * RobotController.getBatteryVoltage();
        m_armSim.setInput(appliedVolts);
        m_armSim.update(SimConstants.k_simPeriodic);

        // Simulate battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    }
}
