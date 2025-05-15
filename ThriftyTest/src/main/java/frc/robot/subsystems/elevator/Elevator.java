// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Seconds;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralLevel;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import frc.robot.subsystems.PassiveSubsystem;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends PassiveSubsystem {
  // we want to have a logger, even if we're not using it... yet
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(Elevator.class);

  private final ElevatorIO m_io;
  private ElevatorIOInputs m_inputs;

  private final Debouncer m_debouncer =
      new Debouncer(ElevatorConstants.kRangeDebounceTime.in(Seconds));

  private ElevatorState m_reference = ElevatorState.Stow;

  private Trigger m_prefireReq = new Trigger(() -> false);

  public Elevator() {
    super();
    if (Robot.isReal()) {
      m_io = new ElevatorIOHardware();
    } else {
      m_io = new ElevatorIOSim();
    }
    m_inputs = new ElevatorIOInputs();
    SmartDashboard.putData("Elevator/Lazy Zero",
        runOnce(m_io::calibrateZero).ignoringDisable(true));
  }

  private void setPosition(ElevatorState state) {
    take();
    // calculate goal we should go to
    double goal = state.position();
    if (RobotObserver.getNoElevatorZone()
        && (m_inputs.position > ElevatorConstants.kUnsafeRange || goal > ElevatorConstants.kUnsafeRange)) {
      // either trying to reach (or already at) a no-go state given our current position
      return;
    }
    // floor values for the goal between our two extrema for their positions
    goal = Math.min(goal, ElevatorConstants.kForwardSoftLimit);
    goal = Math.max(goal, ElevatorConstants.kReverseSoftLimit);
    m_io.setPosition(goal);
    m_reference = state;
  }

  public Trigger ready() {
    return new Trigger(
        () -> Math.abs(m_reference.position() - m_inputs.position) < ElevatorConstants.kTolerance);
  }

  public Trigger ready(ElevatorState state) {
    return new Trigger(() -> {
      if (m_reference.equals(state)) {
        return ready().getAsBoolean();
      }
      return false;
    });
  }

  public Trigger ready(CoralLevel level) {
    return ready(level.toElevatorState());
  }

  private boolean atZero() {
    return m_debouncer.calculate(m_inputs.zeroCANrangeDetected);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    SmartDashboard.putNumber("Elevator/Position", m_inputs.position);
    SmartDashboard.putString("Elevator/Reference", m_reference.toString());
    SmartDashboard.putBoolean("Elevator/Prefire", m_prefireReq.getAsBoolean());
    SmartDashboard.putBoolean("Elevator/Ready", ready().getAsBoolean());
  }

  /**
   * Whether or not the elevator is above the "safe" range
   */
  public Trigger unsafe() {
    return new Trigger(() -> m_inputs.position > ElevatorConstants.kUnsafeRange
        || m_reference.position() > ElevatorConstants.kUnsafeRange);
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
        runOnce(() -> setPosition(state)),
        Commands.waitUntil(ready()))
        .withName(state.toString());
  }

  public Command go(CoralLevel level) {
    return go(level.toElevatorState());
  }

  /**
   * Automatically zeroes the elevator.
   */
  public Command autoZero() {
    return Commands.waitUntil(this::atZero).deadlineFor(
        Commands.sequence(
            go(ElevatorState.Zero),
            runOnce(m_io::disableLimits),
            runOnce(() -> m_io.setVoltage(ElevatorConstants.kZeroVoltage))))

        .finallyDo(m_io::enableLimits)
        .finallyDo(interrupted -> {
          if (!interrupted) {
            m_io.calibrateZero();
          }
        })
        .withName("Autozero");
  }

  public void setPrefireRequirement(Trigger trigger) {
    m_prefireReq = trigger;
  }
}
