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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Robot;
import frc.robot.RobotObserver;

public class Elevator extends PassiveSubsystem {
  // we want to have a logger, even if we're not using it... yet
  private final Logger m_logger = LoggerFactory.getLogger(Elevator.class);

  private final TalonFX m_elevatorLeft = new TalonFX(IDConstants.elevatorLeft, "*");
  private final TalonFX m_elevatorRight = new TalonFX(IDConstants.elevatorRight, "*");

  private final CANrange m_CANrange = new CANrange(IDConstants.elevatorCANrange);

  private final Debouncer m_debouncer =
      new Debouncer(ElevatorConstants.kRangeDebounceTime.in(Seconds));

  private double m_position;
  private double m_reference;

  private ElevatorSim m_elevatorSim;
  private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60Foc(2); // 2 motors (left and right)

  private Mechanism2d m_mechVisual;
  private MechanismRoot2d m_mechRoot;
  private MechanismLigament2d m_elevatorArm;

  private double m_speed;
  private boolean m_speedChanged;

  private Trigger m_prefireReq = new Trigger(() -> false);

  public Elevator() {
    super();
    configMotor();
    configCANrange();
    configSim();
    SmartDashboard.putData("Lazy Zero Elevator",
        runOnce(this::calibrateZero).ignoringDisable(true));
  }

  private void configCANrange() {
    m_CANrange.getConfigurator().apply(
        ElevatorConstants.kCANrangeConfig,
        RobotConstants.globalCanTimeout.in(Seconds));
  }

  private void configMotor() {
    m_elevatorRight.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
    m_elevatorLeft.getConfigurator().apply(ElevatorConstants.motorConfig, 0.2);
    Follower follower = new Follower(
        IDConstants.elevatorRight,
        ElevatorConstants.invertLeftMotorFollower);
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
        ElevatorConstants.reverseSoftLimit);

    m_mechVisual = new Mechanism2d(1, 12); // Width/height in meters
    m_mechRoot = m_mechVisual.getRoot("ElevatorRoot", 0.5, 0.0); // Center at (0.5, 0)
    m_elevatorArm = m_mechRoot.append(new MechanismLigament2d("ElevatorArm", 0.0, 90)); // Start at
                                                                                        // 0.1m
                                                                                        // height
    SmartDashboard.putData("Elevator Visualization", m_mechVisual);
    if (RobotBase.isSimulation()) {
      // in simulation, we want to emulate the effect produced by
      // using an encoder offset (i.e. we start at 0).
      m_elevatorRight.setPosition(0.0);
    }
  }

  private final DynamicMotionMagicVoltage control = new DynamicMotionMagicVoltage(0, 0, 0, 0);

  private void setPosition(double goal) {
    take();
    if (RobotObserver.getNoElevatorZone()
        && (m_position > ElevatorConstants.unsafeRange || goal > ElevatorConstants.unsafeRange)) {
      return;
    }
    // floor values for the goal between our two extrema for their positions
    goal = Math.min(goal, ElevatorConstants.forwardSoftLimit);
    goal = Math.max(goal, ElevatorConstants.reverseSoftLimit);
    m_elevatorRight.setControl(control
        .withPosition(goal)
        .withVelocity(ElevatorConstants.maxSpeedUp)
        .withAcceleration(ElevatorConstants.maxAccelerationUp)
        .withJerk(ElevatorConstants.maxJerkUp)
        .withSlot(0));
    m_reference = goal;
  }

  private void setPosition(ElevatorState state) {
    setPosition(state.position());
  }

  public Trigger ready() {
    return new Trigger(() -> {
      if (Robot.isSimulation())
        return true;
      boolean at = Math.abs(m_reference - m_position) < ElevatorConstants.tolerance;
      m_logger.debug("Setpoint: {}", at);
      return at;
    });
  }

  public double getReference() {
    return m_reference;
  }

  public double getPosition() {
    return m_position;
  }

  private double getPositionUncached() {
    if (RobotBase.isReal()) {
      return m_elevatorRight.getPosition().getValueAsDouble();
    } else {
      return m_elevatorSim.getPositionMeters();
    }
  }

  private void prepZero() {
    m_elevatorRight.getConfigurator().apply(new SoftwareLimitSwitchConfigs());
    m_elevatorRight.setControl(new DutyCycleOut(ElevatorConstants.manualDownSpeed)
        .withLimitReverseMotion(false).withIgnoreHardwareLimits(true));
  }

  private void calibrateZero() {
    m_elevatorRight.setPosition(0.0, 0.2);
  }

  private void enableLimits() {
    m_elevatorRight.getConfigurator().apply(ElevatorConstants.motorConfig.SoftwareLimitSwitch);
  }

  private boolean atZero() {
    return m_debouncer.calculate(m_CANrange.getIsDetected().getValue());
  }

  private void goDownNoStopping() {
    m_elevatorRight.setPosition(1); // TODO: Why is this line here?
    m_logger.warn("unhandled todo!");
    m_elevatorRight.set(ElevatorConstants.manualDownSpeed);
  }

  @Override
  public void periodic() {
    m_position = getPositionUncached();
    SmartDashboard.putNumber("Elevator position", m_position);
    SmartDashboard.putNumber("Elevator reference", m_reference);
    SmartDashboard.putBoolean("prefire?", m_prefireReq.getAsBoolean());

    if (m_speedChanged) {
      m_elevatorRight.setControl(new DutyCycleOut(m_speed));
      m_speedChanged = false;
    }

    SmartDashboard.putBoolean("ELEVATOR AT POSITION", ready().getAsBoolean());
  }

  /**
   * Whether or not the elevator is above the "safe" range
   */
  public Trigger unsafe() {
    return new Trigger(() -> getPosition() > ElevatorConstants.unsafeRange
        || m_reference > ElevatorConstants.unsafeRange);
  }

  protected void passive() {
    if (m_prefireReq.getAsBoolean()) {
      setPosition(ElevatorState.L2);
    } else {
      setPosition(ElevatorState.Stow);
    }
  }

  public Command go(ElevatorState state) {
    return Commands.sequence(
        runOnce(() -> setPosition(state.position())),
        Commands.waitUntil(ready()));
  }

  public Command go(CoralLevel level) {
    switch (level) {
      case L1:
        return go(ElevatorState.L1);
      case SecondaryL1:
        return go(ElevatorState.SecondaryL1);
      case L2:
        return go(ElevatorState.L2);
      case L3:
        return go(ElevatorState.L3);
      case L4:
        return go(ElevatorState.L4);
      default:
        return Commands.none();
    }
  }

  /**
   * Automatically zeroes the elevator.
   */
  public Command autoZero() {
    return Commands.sequence(
        runOnce(this::prepZero),
        runOnce(this::goDownNoStopping),
        Commands.waitUntil(this::atZero))

        .finallyDo(this::enableLimits)
        .finallyDo(interrupted -> {
          if (!interrupted) {
            calibrateZero();
          }
        });
  }

  public void setPrefireRequirement(Trigger trigger) {
    m_prefireReq = trigger;
  }
}
