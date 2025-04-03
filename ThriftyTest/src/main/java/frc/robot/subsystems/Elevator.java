// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Robot;
import frc.robot.RobotObserver;

public class Elevator extends SubsystemBase {
    // we want to have a logger, even if we're not using it... yet
    private final Logger m_logger = LoggerFactory.getLogger(Elevator.class);

    private final TalonFX m_elevatorLeft = new TalonFX(IDConstants.elevatorLeft, "*");
    private final TalonFX m_elevatorRight = new TalonFX(IDConstants.elevatorRight, "*");

    private final CANrange m_CANrange = new CANrange(IDConstants.elevatorCANrange);

    private final Debouncer m_debouncer = new Debouncer(ElevatorConstants.kRangeDebounceTime.in(Seconds));

    private double m_position;
    private double m_velocity;

    private double m_reference;

    private ElevatorSim m_elevatorSim;
    private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60Foc(2); // 2 motors (left and right)

    private Mechanism2d m_mechVisual;
    private MechanismRoot2d m_mechRoot;
    private MechanismLigament2d m_elevatorArm;

    private boolean m_taken;

    private double m_speed;
    private boolean m_speedChanged;

    public Elevator() {
        configMotor();
        configCANrange();
        configSim();
        m_taken = false;
    }

    private void configCANrange() {
        m_CANrange.getConfigurator().apply(ElevatorConstants.kCANrangeConfig, RobotConstants.globalCanTimeout.in(Seconds));
    }

    private void configMotor() {
        m_elevatorRight.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
        m_elevatorLeft.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
        Follower follower = new Follower(IDConstants.elevatorRight, ElevatorConstants.invertLeftMotorFollower);
        m_elevatorLeft.setControl(follower);
        m_elevatorRight.setPosition(0);
    }

    private void configSim() {
        m_elevatorSim = new ElevatorSim(
                ElevatorConstants.stateSpacePlant,
                m_elevatorGearbox,
                ElevatorConstants.reverseSoftLimit,
                ElevatorConstants.forwardSoftLimit,
                true,
                ElevatorConstants.reverseSoftLimit
        );

        m_mechVisual = new Mechanism2d(1, 12); // Width/height in meters
        m_mechRoot = m_mechVisual.getRoot("ElevatorRoot", 0.5, 0.0); // Center at (0.5, 0)
        m_elevatorArm = m_mechRoot.append(new MechanismLigament2d("ElevatorArm", 0.0, 90)); // Start at 0.1m height
        SmartDashboard.putData("Elevator Visualization", m_mechVisual);
        if (RobotBase.isSimulation()) {
            // in simulation, we want to emulate the effect produced by
            // using an encoder offset (i.e. we start at 0).
            m_elevatorRight.setPosition(0.0);
        }
    }

    // private final MotionMagicVoltage control = new MotionMagicVoltage(0);
    private final DynamicMotionMagicVoltage control = new DynamicMotionMagicVoltage(0,0,0,0);

    public void setPosition(double goal) {
        m_taken = true;
        if (RobotObserver.getNoElevatorZone() && (m_position > ElevatorConstants.unsafeRange || goal > ElevatorConstants.unsafeRange)) {
            return;
        }
        // floor values for the goal between our two extrema for their positions
        goal = Math.min(goal, ElevatorConstants.forwardSoftLimit);
        goal = Math.max(goal, ElevatorConstants.reverseSoftLimit);
        if (goal >= getPosition() || true) {
            m_elevatorRight.setControl(control
                .withPosition(goal)
                .withVelocity(ElevatorConstants.maxSpeedUp)
                .withAcceleration(ElevatorConstants.maxAccelerationUp)
                .withJerk(ElevatorConstants.maxJerkUp)
                .withSlot(0));
        } else {
            m_elevatorRight.setControl(control
                .withPosition(goal)
                .withVelocity(ElevatorConstants.maxSpeedDown)
                .withAcceleration(ElevatorConstants.maxAccelerationDown)
                .withJerk(ElevatorConstants.maxJerkDown)
                .withSlot(1));
        }

        m_reference = goal;
    }

    public void setSpeed(double speed) {
        m_speedChanged = (speed != m_speed);
        m_speed = speed;
    }

    public void setGroundIntake() {
        setPosition(ElevatorConstants.groundIntake);
    }

    public void setHighGroundIntake() {
        setPosition(ElevatorConstants.highGroundIntake);
    }

    public void setStow() {
        setPosition(ElevatorConstants.stow);
    }

    public void setEject() {
        setPosition(ElevatorConstants.eject);
    }

    public void setProcessor() {
        setPosition(ElevatorConstants.processor);
    }

    public void setZero() {
        setPosition(0);
    }

    public void setL1() {
        setPosition(ElevatorConstants.L1);
    }

    public void setL2() {
        setPosition(ElevatorConstants.L2);
    }

    public void setL3() {
        setPosition(ElevatorConstants.L3);
    }

    public void setL4() {
        setPosition(ElevatorConstants.L4);
    }

    public void setPrep() {
        // no compensation
        setPosition(ElevatorConstants.prep);
    }

    public void setReefLower() {
        setPosition(ElevatorConstants.reefLower);
    }

    public void setReefUpper() {
        setPosition(ElevatorConstants.reefUpper);
    }

    public void setNet() {
        setPosition(ElevatorConstants.net);
    }

    public void stop() {
        setPosition(m_position);
    }

    public void setLevel(int level) {
        switch (level) {
            case 1 -> setL1();
            case 2 -> setL2();
            case 3 -> setL3();
            case 4 -> setL4();
            case 5 -> setNet();
            case 0 -> setStow();
            default -> setStow();
        }
    }

    public boolean atSetpoint() {
        if (Robot.isSimulation()) return true;
        return Math.abs(m_reference - m_position) < ElevatorConstants.tolerance;
    }

    public double getReference() {
        return m_reference;
    }

    public double getPosition() {
        return m_position;
    }

    public double getVelocity() {
        return m_velocity;
    }

    private double getPositionUncached() {
        if (RobotBase.isReal()) {
            return m_elevatorRight.getPosition().getValueAsDouble();
        } else {
            return m_elevatorSim.getPositionMeters();
        }
    }

    private double getVelocityUncached() {
        if (RobotBase.isReal()) {
            return m_elevatorRight.getVelocity().getValueAsDouble();
        } else {
            return m_elevatorSim.getVelocityMetersPerSecond();
        }
    }

    public void prepZero() {
        m_elevatorRight.getConfigurator().apply(new SoftwareLimitSwitchConfigs());
        m_elevatorRight.setControl(new DutyCycleOut(ElevatorConstants.manualDownSpeed)
                .withLimitReverseMotion(false)
                .withIgnoreHardwareLimits(true));
    }

    public void zeroElevator() {
        m_elevatorRight.setPosition(0.0, 0.2);
    }

    public void enableLimits() {
        m_elevatorRight.getConfigurator().apply(ElevatorConstants.motorConfig.SoftwareLimitSwitch);
    }

    public boolean atZero() {
        return m_debouncer.calculate(m_CANrange.getIsDetected().getValue());
        // return m_elevatorRight.getSupplyCurrent().getValueAsDouble() >= ElevatorConstants.k_zeroCurrentThreshold;
    }

    public void goDownNoStopping() {
        m_elevatorRight.setPosition(1);
        m_elevatorRight.set(ElevatorConstants.manualDownSpeed);
    }

    @Override
    public void periodic() {
        m_position = getPositionUncached();
        m_velocity = getVelocityUncached();

        if (m_speedChanged) {
            m_elevatorRight.setControl(new DutyCycleOut(m_speed));
            m_speedChanged = false;
        }

        SmartDashboard.putBoolean("ELEVATOR AT POSITION", atSetpoint());
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation with the motor voltage
        double appliedVolts = m_elevatorRight.get() * RobotController.getBatteryVoltage() * 2;

        m_elevatorSim.setInput(appliedVolts);
        m_elevatorSim.update(SimConstants.k_simPeriodic);

        m_position = getPositionUncached();
        m_velocity = getVelocityUncached();

        // Update the simulated encoder values
        m_elevatorRight.getSimState().setRawRotorPosition(m_position);
        m_elevatorRight.getSimState().setRotorVelocity(m_velocity);

        m_elevatorArm.setLength(m_position + 0.1); // Offset to avoid overlapping with root
        
        // Simulate battery voltage
        double volts = BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps());
        RoboRioSim.setVInVoltage(volts);
    }

    public boolean elevatorUp() {
        return getPosition() > ElevatorConstants.unsafeRange;
    }

    public void release() {
        m_taken = false;
    }

    public boolean taken() {
        return m_taken;
    }
}
