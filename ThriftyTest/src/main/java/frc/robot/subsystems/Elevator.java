// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.RobotContainer.CoralLocationPresets;
import frc.robot.utils.RunOnChange;

public class Elevator extends SubsystemBase {
    public enum ElevatorSetpoints {
        GROUND(ElevatorConstants.ground), 
        STOW(ElevatorConstants.stow), 
        PROCESSOR(ElevatorConstants.processor), 
        HIGHGROUND(ElevatorConstants.highGround), 
        L1(ElevatorConstants.L1), 
        L2(ElevatorConstants.L2), 
        ALGAE_L2(ElevatorConstants.algaeL2), 
        L3(ElevatorConstants.L3), 
        ALGAE_L3(ElevatorConstants.algaeL3), 
        L4(ElevatorConstants.L4), 
        NET(ElevatorConstants.net);

        private double value;

        private ElevatorSetpoints(double value) {
            this.value = value;
        }

        public static ElevatorSetpoints fromAlgaePreset(AlgaeLocationPresets preset) {
            ElevatorSetpoints setpoint = ElevatorSetpoints.STOW;
            switch (preset) {
                case ALGAE_L2 -> setpoint = ElevatorSetpoints.ALGAE_L2;
                case ALGAE_L3 -> setpoint = ElevatorSetpoints.ALGAE_L3;
                case GROUND -> setpoint = ElevatorSetpoints.GROUND;
                case NET -> setpoint = ElevatorSetpoints.NET;
                case PROCESSOR -> setpoint = ElevatorSetpoints.PROCESSOR;
                case HIGHGROUND -> setpoint = ElevatorSetpoints.HIGHGROUND;
            }
            return setpoint;
        }

        public static ElevatorSetpoints fromCoralPreset(CoralLocationPresets preset) {
            ElevatorSetpoints setpoint = ElevatorSetpoints.STOW;
            switch (preset) {
                case L1 -> setpoint = ElevatorSetpoints.L1;
                case L2 -> setpoint = ElevatorSetpoints.L2;
                case L3 -> setpoint = ElevatorSetpoints.L3;
                case L4 -> setpoint = ElevatorSetpoints.L4;
                case LEFT_INTAKE -> setpoint = ElevatorSetpoints.STOW;
                case RIGHT_INTAKE -> setpoint = ElevatorSetpoints.STOW;
            }
            return setpoint;
        }
    }

    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(Elevator.class);

    private final TalonFX m_elevatorLeft = new TalonFX(IDConstants.elevatorLeft, "*"); // CANivore
    private final TalonFX m_elevatorRight = new TalonFX(IDConstants.elevatorRight, "*"); // CANivore
    private final CANcoder m_cancoder = new CANcoder(IDConstants.elevatorEncoder);

    private StatusSignal<Angle> m_position;
    private StatusSignal<AngularVelocity> m_velocity;
    
    private ElevatorSetpoints m_reference;

    // private RunOnChange<Double> changeVolts;
    private RunOnChange<Double> changeSetpoint;

    private Alert encoderNotConfigured = new Alert("Elevator Cancoder on ID " + IDConstants.elevatorEncoder + " did not successfully connect", AlertType.kError);

    public Elevator() {
        configEncoder();
        configMotor();
        configSim();
        m_reference = ElevatorSetpoints.STOW;
        // changeVolts = new RunOnChange<>(this::writeToMotors, 0.0);
        changeSetpoint = new RunOnChange<>(this::writePositionToMotors, ElevatorConstants.stow);

        // Disables status signals communicated that don't need to be
        // ParentDevice.optimizeBusUtilizationForAll(m_elevatorLeft, m_elevatorRight, m_cancoder);
        BaseStatusSignal.waitForAll(0.02, m_position, m_velocity);
    }

    private void configEncoder() {
        m_cancoder.clearStickyFaults();
        // If cancoder.apply returns an error, throw SmartDashboard alert
        encoderNotConfigured.set(!m_cancoder.getConfigurator().apply(ElevatorConstants.encoderConfig, 0.2).isOK());
    }

    private void configMotor() {
        m_elevatorRight.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
        m_elevatorLeft.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);

        // Enabling so we can get position and velocity values
        m_position = m_elevatorRight.getPosition();
        m_position.setUpdateFrequency(0.02);
        
        m_velocity = m_elevatorRight.getVelocity();
        m_velocity.setUpdateFrequency(0.02);

        // Required for master motor
        m_elevatorRight.getDutyCycle().setUpdateFrequency(0.02);
        m_elevatorRight.getMotorVoltage().setUpdateFrequency(0.02);
        m_elevatorRight.getTorqueCurrent().setUpdateFrequency(0.02);

        Follower follower = new Follower(IDConstants.elevatorRight, ElevatorConstants.invertLeftMotorFollower);
        m_elevatorLeft.setControl(follower);
    }

    // private void writeToMotors(double voltage) {
    //     m_elevatorRight.setVoltage(voltage);
    // }

    // private final MotionMagicVoltage control = new MotionMagicVoltage(0);
    private final DynamicMotionMagicVoltage control = new DynamicMotionMagicVoltage(0, 0, 0, 0);

    private void writePositionToMotors(double setpoint) {
        m_elevatorRight.setControl(control.withPosition(setpoint));
        
        if (setpoint >= getPosition()) {
            m_elevatorRight.setControl(control
                .withPosition(setpoint)
                .withVelocity(ElevatorConstants.maxSpeedUp)
                .withAcceleration(ElevatorConstants.maxSpeedUp * ElevatorConstants.accelerationMultiplierUp)
                .withJerk(ElevatorConstants.maxSpeedUp * ElevatorConstants.accelerationMultiplierUp * 10)
                .withSlot(0));
        } else {
            m_elevatorRight.setControl(control
                .withPosition(setpoint)
                .withVelocity(ElevatorConstants.maxSpeedDown)
                .withAcceleration(ElevatorConstants.maxSpeedDown * ElevatorConstants.accelerationMultiplierUp)
                .withJerk(ElevatorConstants.maxSpeedDown * ElevatorConstants.accelerationMultiplierUp * 10)
                .withSlot(1));
        }
    }

    private void setPosition(double goal) {
        changeSetpoint.accept(goal);        
    }

    // public void setVoltage(double speed) {
    //     changeVolts.accept(speed);
    // }

    public void set(ElevatorSetpoints setpoint) {
        m_reference = setpoint;
        setPosition(setpoint.value);
    }

    public void stow() {
        setPosition(ElevatorConstants.stow);
    }

    public void stop() {
        setPosition(getPosition());
    }

    public void setLevel(int level) {
        switch (level) {
            case 1 -> set(ElevatorSetpoints.L1);
            case 2 -> set(ElevatorSetpoints.L2);
            case 3 -> set(ElevatorSetpoints.L3);
            case 4 -> set(ElevatorSetpoints.L4);
            default -> set(ElevatorSetpoints.STOW);
        }
    }

    public double getError() {
        return getReference().value - getPosition();
    }

    public boolean atSetpoint() {
        return Math.abs(getError()) < ElevatorConstants.tolerance;
    }

    public ElevatorSetpoints getReference() {
        return m_reference;
    }

    public double getPosition() {
        if (Robot.isReal()) {
            return m_position.getValueAsDouble();
        } else {
            return m_elevatorSim.getPositionMeters();
        }
    }

    public double getLatencyCompensatedPosition() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(m_position, m_velocity);
    }

    public double getVelocity() {
        if (Robot.isReal()) {
            return m_velocity.getValueAsDouble();
        } else {
            return m_elevatorSim.getVelocityMetersPerSecond();
        }
    }

    private void update() {
        if (Robot.isReal()) {
            BaseStatusSignal.refreshAll(m_position, m_velocity);
        }

        // changeVolts.resolveIfChange();
        changeSetpoint.resolveIfChange();
    }

    private void log() {
        SmartDashboard.putString("Elevator Goal", getReference().toString());
        SmartDashboard.putNumber("Elevator Goal Value", getReference().value);
        SmartDashboard.putBoolean("Elevator At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Position", getPosition());
    }

    @Override
    public void periodic() {
        update();
        log();
    }


    // SIMULATION  

    private ElevatorSim m_elevatorSim;
    private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60(2); // 2 motors (left and right)

    private Mechanism2d m_mechVisual;
    private MechanismRoot2d m_mechRoot;
    private MechanismLigament2d m_elevatorArm;

    private void configSim() {
        m_elevatorSim = new ElevatorSim(
                ElevatorConstants.stateSpacePlant,
                m_elevatorGearbox,
                ElevatorConstants.reverseSoftLimit,
                ElevatorConstants.forwardSoftLimit,
                true,
                0.0
        );

        m_mechVisual = new Mechanism2d(1, 12); // Width/height in meters
        m_mechRoot = m_mechVisual.getRoot("ElevatorRoot", 0.5, 0.0); // Center at (0.5, 0)
        m_elevatorArm = m_mechRoot.append(new MechanismLigament2d("ElevatorArm", 0.0, 90)); // Start at 0.1m height
        SmartDashboard.putData("Elevator Visualization", m_mechVisual);
    }  

    @Override
    public void simulationPeriodic() {
        update();
        
        double appliedVolts = getError() * 24;

        m_elevatorSim.setInput(appliedVolts);
        m_elevatorSim.update(SimConstants.k_simPeriodic);
        m_elevatorArm.setLength(getPosition());

        log();
    }
}
