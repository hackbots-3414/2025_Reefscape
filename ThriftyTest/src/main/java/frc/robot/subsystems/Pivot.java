// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.stateSpace.StateSpaceController;

public class Pivot extends SubsystemBase {
    private final TalonFX m_pivot = new TalonFX(IDConstants.pivotMotor);
    private final CANcoder m_cancoder = new CANcoder(IDConstants.pivotEncoder);

    private StateSpaceController<N2, N1, N2> m_controller;

    private double m_position;
    private double m_velocity;

    private boolean m_stateSpaceEnabled;

    private SingleJointedArmSim m_armSim;
    private double m_simPosition = 0.0; // Simulated position in meters
    private double m_simVelocity = 0.0; // Simulated velocity in meters per second
    private final DCMotor m_gearbox = DCMotor.getFalcon500(1); // 2 motors (left and right)

    private Mechanism2d m_mechVisual;
    private MechanismRoot2d m_mechRoot;
    private MechanismLigament2d m_armLigament;

    private double m_speed;
    private boolean m_speedChanged;

    public Pivot() {
        configEncoder();
        configMotor();
        configStateSpace();
        configSim();
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
                .append(new MechanismLigament2d("Arm", PivotConstants.armLength, Math.toDegrees(m_simPosition)));
        SmartDashboard.putData("Pivot Arm Visualization", m_mechVisual);
    }

    private void configStateSpace() {
        Vector<N2> initialState = getOutput();
        m_controller = new StateSpaceController<>(
            PivotConstants.stateSpaceConfig,
            this::getOutput,
            this::applyInput,
            initialState
        );
        enableStateSpace();
    }

    private Vector<N2> getOutput() {
        if (RobotBase.isReal()) {
            return VecBuilder.fill(
                getPositionUncached(),
                getVelocityUncached()
            );
        } else {
            return VecBuilder.fill(
                getSimPositionUncached(),
                getSimVelocityUncached()
            );
        }
    }

    private void applyInput(Vector<N1> inputs) {
        if (!m_stateSpaceEnabled) return;

        VoltageOut config = new VoltageOut(0);
        double volts = inputs.get(0);

        m_pivot.setControl(config.withOutput(volts));
    }

    public void setM_position(double goal) {
        m_controller.setReference(VecBuilder.fill(goal, 0.0));
    }

    public void setSpeed(double speed) {
        m_speedChanged = (speed != m_speed);
        m_speed = speed;
    }

    public void enableStateSpace() {
        m_controller.setReference(getOutput());
        m_stateSpaceEnabled = true;
    }

    public void disableStateSpace() {
        m_stateSpaceEnabled = false;
    }

    public void setStow() {
        setM_position(PivotConstants.stow);
    }

    public void setProcessor() {
        setM_position(PivotConstants.processor);
    }

    public void setNet() {
        setM_position(PivotConstants.net);
    }

    public void setGroundPickup() {
        setM_position(PivotConstants.groundPickup);
    }

    public void setReefPickup() {
        setM_position(PivotConstants.reefPickup);
    }

    public void setReefExtract() {
        setM_position(PivotConstants.reefExtract);
    }
    
    public void stop() {
        setM_position(m_position);
    }

    public double getM_position() {
        return m_position;
    }

    public double getM_velocity() {
        return m_velocity;
    }
    
    public boolean atSetpoint() {
        return m_controller.isAtSetpoint();
    }
    
    private double getPositionUncached() {
        return m_cancoder.getPosition().getValueAsDouble();
    }

    private double getVelocityUncached() {
        return m_cancoder.getVelocity().getValueAsDouble();
    }

    private double getSimPositionUncached() {
        return m_armSim.getAngleRads();
    }

    private double getSimVelocityUncached() {
        return m_armSim.getVelocityRadPerSec();
    }

    @Override
    public void periodic() {
        m_position = getPositionUncached();
        m_velocity = getVelocityUncached();

        m_armLigament.setAngle(Math.toDegrees(m_simPosition));

        if (m_speedChanged && !m_stateSpaceEnabled) {
            m_pivot.setControl(new DutyCycleOut(m_speed));
            m_speedChanged = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation with the motor voltage
        double appliedVolts = m_pivot.get() * RobotController.getBatteryVoltage();
        m_armSim.setInput(appliedVolts);
        m_armSim.update(SimConstants.k_simPeriodic);

        // Update simulated angle and angular velocity
        m_simPosition = getSimPositionUncached();
        m_simVelocity = getSimVelocityUncached();

        // Update the simulated encoder values
        m_cancoder.getSimState().setRawPosition(m_simPosition / (2 * Math.PI)); // Convert radians to rotations
        m_cancoder.getSimState().setVelocity(m_simVelocity / (2 * Math.PI)); // Convert rad/s to RPM

        // Simulate battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    }
}
