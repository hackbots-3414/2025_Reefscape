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
import frc.robot.Constants.PivotConstants;
import frc.robot.stateSpace.StateSpaceController;

public class Pivot extends SubsystemBase {
    private final TalonFX pivot = new TalonFX(PivotConstants.motorID);
    private final CANcoder cancoder = new CANcoder(PivotConstants.encoderID);

    private StateSpaceController<N2, N1, N2> controller;

    private double position;
    private double velocity;

    private boolean stateSpaceEnabled;

    private SingleJointedArmSim armSim;
    private double simPosition = 0.0; // Simulated position in meters
    private double simVelocity = 0.0; // Simulated velocity in meters per second
    private final DCMotor armGearbox = DCMotor.getFalcon500(1); // 2 motors (left and right)

    private Mechanism2d mechVisual;
    private MechanismRoot2d mechRoot;
    private MechanismLigament2d armLigament;

    private double m_speed;
    private boolean m_speedChanged;

    public Pivot() {
        configEncoder();
        configMotor();
        configStateSpace();
        configSim();
    }

    private void configEncoder() {
        cancoder.clearStickyFaults();
        cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);
        cancoder.getConfigurator().apply(PivotConstants.encoderConfig, 0.2);
    }

    private void configMotor() {
        pivot.getConfigurator().apply(PivotConstants.motorConfig, 0.2);
    }

    private void configSim() {
        armSim = new SingleJointedArmSim(
                PivotConstants.stateSpacePlant,
                armGearbox,
                PivotConstants.gearRatio,
                PivotConstants.armLength,
                PivotConstants.radiansAtZero,
                PivotConstants.radiansAtMax,
                true, // Add noise for realism
                PivotConstants.stow // Starting angle
        );

        mechVisual = new Mechanism2d(1.0, 1.0); // Width/height in meters
        mechRoot = mechVisual.getRoot("ArmRoot", 0.5, 0.0); // Center at (0.5, 0)
        armLigament = mechRoot
                .append(new MechanismLigament2d("Arm", PivotConstants.armLength, Math.toDegrees(simPosition)));
        SmartDashboard.putData("Pivot Arm Visualization", mechVisual);
    }

    private void configStateSpace() {
        Vector<N2> initialState = getOutput();
        controller = new StateSpaceController<>(PivotConstants.stateSpaceConfig, this::getOutput, this::applyInput,
                initialState);
        enableStateSpace();
    }

    private Vector<N2> getOutput() {
        if (RobotBase.isSimulation()) {
            return VecBuilder.fill(simPosition, simVelocity);
        } else {
            return VecBuilder.fill(position, velocity);
        }
    }

    private void applyInput(Vector<N1> inputs) {
        if (!stateSpaceEnabled) return;

        VoltageOut config = new VoltageOut(0);
        double volts = inputs.get(0);

        pivot.setControl(config.withOutput(volts));
    }

    public void setPosition(double goal) {
        controller.setReference(VecBuilder.fill(goal, 0.0));
    }

    public void setSpeed(double speed) {
        m_speedChanged = (speed == m_speed);
        m_speed = speed;
    }

    public void enableStateSpace() {
        controller.setReference(getOutput());
        stateSpaceEnabled = true;
    }

    public void disableStateSpace() {
        stateSpaceEnabled = false;
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

    public void stop() {
        setPosition(position);
    }

    public double getPosition() {
        return position;
    }

    public boolean atSetpoint() {
        return controller.isAtSetpoint();
    }

    @Override
    public void periodic() {
        position = cancoder.getPosition().getValueAsDouble();
        velocity = cancoder.getVelocity().getValueAsDouble();

        armLigament.setAngle(Math.toDegrees(simPosition));

        if (m_speedChanged) {
            pivot.setControl(new DutyCycleOut(m_speed));
            m_speedChanged = false;
        }
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation with the motor voltage
        double appliedVolts = pivot.get() * RobotController.getBatteryVoltage();
        armSim.setInput(appliedVolts);
        armSim.update(0.02); // Update every 20ms (standard loop time)

        // Update simulated angle and angular velocity
        simPosition = armSim.getAngleRads();
        simVelocity = armSim.getVelocityRadPerSec();

        // Update the simulated encoder values
        cancoder.getSimState().setRawPosition(simPosition / (2 * Math.PI)); // Convert radians to rotations
        cancoder.getSimState().setVelocity(simVelocity / (2 * Math.PI)); // Convert rad/s to RPM

        // Simulate battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }
}
