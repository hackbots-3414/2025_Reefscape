// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.utils.RunOnChange;

public class Pivot extends SubsystemBase {
    public enum PivotSetpoints {
        GROUND(PivotConstants.groundPickup), 
        STOW(PivotConstants.stow), 
        PROCESSOR(PivotConstants.processor), 
        REEF_PICKUP(PivotConstants.reefPickup), 
        REEF_EXTRACT(PivotConstants.reefExtract), 
        NET(PivotConstants.net);

        private double value;

        private PivotSetpoints(double value) {
            this.value = value;
        }

        public static PivotSetpoints fromAlgaePreset(AlgaeLocationPresets preset) {
            PivotSetpoints setpoint = PivotSetpoints.STOW;
            switch (preset) {
                case ALGAE_L2 -> setpoint = PivotSetpoints.REEF_PICKUP;
                case ALGAE_L3 -> setpoint = PivotSetpoints.REEF_PICKUP;
                case GROUND -> setpoint = PivotSetpoints.GROUND;
                case NET -> setpoint = PivotSetpoints.NET;
                case PROCESSOR -> setpoint = PivotSetpoints.PROCESSOR;
                case HIGHGROUND -> setpoint = PivotSetpoints.GROUND;
            }
            return setpoint;
        }
    }

    private final TalonFX m_pivot = new TalonFX(IDConstants.pivot);
    private final CANcoder m_cancoder = new CANcoder(IDConstants.pivotEncoder);

    private StatusSignal<Angle> m_position;
    private StatusSignal<AngularVelocity> m_velocity;

    private PivotSetpoints m_reference;

    private RunOnChange<Double> changeVolts;
    private RunOnChange<Double> changeSetpoint;

    public Pivot() {
        configSim();
        configEncoder();
        configMotor();
        m_reference = PivotSetpoints.STOW;
        changeVolts = new RunOnChange<>(this::writeToMotors, 0.0);
        changeSetpoint = new RunOnChange<>(this::writePositionToMotors, PivotConstants.stow);
        
        // Disables status signals communicated that don't need to be
        ParentDevice.optimizeBusUtilizationForAll(m_pivot, m_cancoder);
        BaseStatusSignal.waitForAll(0.02, m_position, m_velocity);
    }

    private void configEncoder() {
        m_cancoder.clearStickyFaults();
        m_cancoder.getConfigurator().apply(PivotConstants.encoderConfig, 0.2);
    }

    private void configMotor() {
        m_pivot.getConfigurator().apply(PivotConstants.motorConfig, 0.2);

        // Enabling so we can get position and velocity values
        m_position = m_pivot.getPosition();
        m_position.setUpdateFrequency(50);
        
        m_velocity = m_pivot.getVelocity();
        m_velocity.setUpdateFrequency(50);
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
        m_reference = setpoint;
        setPosition(setpoint.value);
    }

    public void stow() {
        setPosition(PivotConstants.stow);
    }
    
    public void stop() {
        setPosition(getPosition());
    }

    public double getError() {
        return getReference().value - getPosition();
    }

    public double getPosition() {
        if (Robot.isReal()) {
            return m_position.getValueAsDouble();
        } else {
            return m_pivotSim.getAngleRads();
        }
    }

    public PivotSetpoints getReference() {
        return m_reference;
    }

    public double getVelocity() {
        if (Robot.isReal()) {
            return m_velocity.getValueAsDouble();
        } else {
            return m_pivotSim.getVelocityRadPerSec();
        }
    }
    
    public boolean atSetpoint() {
        return Math.abs(getError()) < PivotConstants.tolerance;
    }

    private void update() {
        if (Robot.isReal()) {
            BaseStatusSignal.refreshAll(m_position, m_velocity);
        }

        changeVolts.resolveIfChange();
        changeSetpoint.resolveIfChange();
    }

    private void log() {
        SmartDashboard.putString("Pivot Goal", getReference().toString());
        SmartDashboard.putNumber("Pivot Goal Value", getReference().value);
        SmartDashboard.putBoolean("Pivot At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Pivot Position", getPosition());
    }

    @Override
    public void periodic() {
        update();
        log();
    }


    // SIMULATION

    private SingleJointedArmSim m_pivotSim;
    private final DCMotor m_gearbox = DCMotor.getKrakenX60(1); // 2 motors (left and right)

    private Mechanism2d m_mechVisual;
    private MechanismRoot2d m_mechRoot;
    private MechanismLigament2d m_pivotArm;

    private void configSim() {
        m_pivotSim = new SingleJointedArmSim(
                PivotConstants.stateSpacePlant,
                m_gearbox,
                PivotConstants.gearRatio,
                1.0,
                Units.degreesToRadians(-30),
                Units.degreesToRadians(90),
                true, // Add noise for realism
                Units.degreesToRadians(-30)
        );

        m_mechVisual = new Mechanism2d(2.0, 2.0);
        m_mechRoot = m_mechVisual.getRoot("ArmRoot", 0, 0.5);
        m_pivotArm = m_mechRoot.append(new MechanismLigament2d("Arm", 1.0, Units.degreesToRadians(-30)));
        SmartDashboard.putData("Pivot Arm Visualization", m_mechVisual);
    }

    @Override
    public void simulationPeriodic() {
        update();

        double appliedVolts = getError() * 5;

        m_pivotSim.setInput(appliedVolts);
        m_pivotSim.update(SimConstants.k_simPeriodic);
        m_pivotArm.setAngle(getPosition() * 100);

        log();
    }
}
