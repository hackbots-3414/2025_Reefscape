// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralLevel;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import frc.robot.subsystems.PassiveSubsystem;

public class CoralRollers extends PassiveSubsystem {
  private final Logger m_logger = LoggerFactory.getLogger(CoralRollers.class);

  private final TalonFX m_coralLeft = new TalonFX(CoralConstants.kLeftMotorID);
  private final TalonFX m_coralRight = new TalonFX(CoralConstants.kRightMotorID);

  private final CANrange m_frontRange = new CANrange(CoralConstants.kFrontCANrangeID);
  private final CANrange m_upperRange = new CANrange(CoralConstants.kUpperCANrangeID);
  private final CANrange m_innerRange = new CANrange(CoralConstants.kInnerCANrangeID);

  private double m_voltageLeft;
  private double m_voltageRight;

  public CoralRollers() {
    super();
    configMotors();
    configCANrange();
    RobotObserver.setPieceHeldSupplier(holding());
  }

  private void configMotors() {
    m_coralLeft.clearStickyFaults();
    m_coralRight.clearStickyFaults();

    m_coralLeft.getConfigurator().apply(CoralConstants.kMotorConfig);
    m_coralRight.getConfigurator().apply(CoralConstants.kMotorConfig.withMotorOutput(
        CoralConstants.kMotorConfig.MotorOutput.withInverted(CoralConstants.kInvertRight)));
  }

  private void configCANrange() {
    m_frontRange.getConfigurator().apply(CoralConstants.kFrontRangeConfig);
    m_upperRange.getConfigurator().apply(CoralConstants.kUpperRangeConfig);
    m_innerRange.getConfigurator().apply(CoralConstants.kInnerRangeConfig);
  }

  private void setRightVoltage(double voltage) {
    take();
    if (m_voltageRight != voltage) {
      m_coralRight.setVoltage(voltage);
      m_voltageRight = voltage;
    }
  }

  private void setLeftVoltage(double voltage) {
    take();
    if (m_voltageLeft != voltage) {
      m_coralLeft.setVoltage(voltage);
      m_voltageLeft = voltage;
    }
  }

  private void setVoltage(double voltage) {
    setLeftVoltage(voltage);
    setRightVoltage(voltage);
  }

  private void setIntake() {
    setVoltage(CoralConstants.kIntakeVoltage);
  }

  private void setL2Score() {
    m_logger.trace("Setting L2 eject");
    setVoltage(CoralConstants.kL2EjectVoltage);
  }

  private void setL3Score() {
    m_logger.trace("Setting L3 eject");
    setVoltage(CoralConstants.kL3EjectVoltage);
  }

  private void setL4Score() {
    m_logger.trace("Setting L4 eject");
    setVoltage(CoralConstants.kL4EjectVoltage);
  }

  private void setL1Score() {
    setLeftVoltage(CoralConstants.kL1LeftEjectVoltage);
    setRightVoltage(CoralConstants.kL1RightEjectVoltage);
  }

  private boolean getFrontCANrange() {
    if (Robot.isSimulation()) {
      return SmartDashboard.getBoolean("Coral/Front CANrange", false);
    }
    return m_frontRange.getIsDetected().getValue();
  }

  private boolean getUpperCANrange() {
    if (Robot.isSimulation()) {
      return SmartDashboard.getBoolean("Coral/Upper CANrange", false);
    }
    return m_upperRange.getIsDetected().getValue();
  }

  private boolean getInnerCANrange() {
    if (Robot.isSimulation()) {
      return SmartDashboard.getBoolean("Coral/Inner CANrange", false);
    }
    return m_innerRange.getIsDetected().getValue();
  }

  private void stop() {
    setVoltage(0);
  }

  public Trigger present() {
    return new Trigger(() -> getUpperCANrange() || getInnerCANrange() || getFrontCANrange());
  }

  public Trigger holding() {
    return new Trigger(() -> getFrontCANrange() && !getUpperCANrange());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral/Front CANrange", getFrontCANrange());
    SmartDashboard.putBoolean("Coral/Upper CANrange", getUpperCANrange());
    SmartDashboard.putBoolean("Coral/Inner CANrange", getInnerCANrange());
    SmartDashboard.putBoolean("Coral/Holding", holding().getAsBoolean());
    SmartDashboard.putBoolean("Coral/Present", present().getAsBoolean());
  }

  protected void passive() {
    if (present().getAsBoolean() && !holding().getAsBoolean()) {
      setIntake();
    } else {
      stop();
    }
  }

  /**
   * Intakes a game piece. The command ends when the piece is fully in the robot.
   */
  public Command intake() {
    return Commands.sequence(
        runOnce(this::setIntake),
        Commands.waitUntil(holding()))

        .finallyDo(this::stop)
        .unless(holding());
  }

  public Command score(CoralLevel level) {
    return Commands.sequence(
        runOnce(() -> {
          switch (level) {
            case L1, SecondaryL1 -> setL1Score();
            case L2 -> setL2Score();
            case L3 -> setL3Score();
            case L4 -> setL4Score();
          }
        }),
        Commands.waitUntil(holding().negate()))

        .onlyIf(holding());
  }

  /**
   * Ejects a coral piece
   */
  public Command eject() {
    return Commands.sequence(
        runOnce(() -> setVoltage(CoralConstants.kEjectVoltage)),
        Commands.waitUntil(present().negate()))

        .finallyDo(this::stop)
        .onlyIf(present());
  }

}
