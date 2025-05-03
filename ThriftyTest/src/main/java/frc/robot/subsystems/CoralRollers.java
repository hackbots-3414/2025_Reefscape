// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.IDConstants;
import frc.robot.Robot;
import frc.robot.RobotObserver;

public class CoralRollers extends PassiveSubsystem {
  private final Logger m_logger = LoggerFactory.getLogger(CoralRollers.class);

  private final TalonFX m_coralLeft = new TalonFX(IDConstants.coralLeft);
  private final TalonFX m_coralRight = new TalonFX(IDConstants.coralRight);

  private final CANrange m_frontRange = new CANrange(IDConstants.coralCANrange);
  private final CANrange m_upperRange = new CANrange(IDConstants.upperCANrange);
  private final CANrange m_innerRange = new CANrange(IDConstants.innerCANrange);

  private double m_voltage;
  private boolean m_voltageChanged;

  public CoralRollers() {
    super();
    configMotors();
    configDashboard();
    configCANrange();
    RobotObserver.setPieceHeldSupplier(holding());
  }

  private void configMotors() {
    m_coralLeft.clearStickyFaults();
    m_coralRight.clearStickyFaults();

    m_coralLeft.getConfigurator().apply(CoralConstants.motorConfig);
    m_coralRight.getConfigurator().apply(CoralConstants.motorConfig.withMotorOutput(
        CoralConstants.motorConfig.MotorOutput.withInverted(CoralConstants.kInvertRight)));
  }

  private void configDashboard() {
    if (Robot.isReal()) {
      // NOTHING YET
    } else {
      SmartDashboard.putBoolean("Coral Override", false);
    }
  }

  private void configCANrange() {
    m_frontRange.getConfigurator().apply(CoralConstants.frontRangeConfig);
    m_upperRange.getConfigurator().apply(CoralConstants.upperRangeConfig);
    m_innerRange.getConfigurator().apply(CoralConstants.innerRangeConfig);
  }

  private void setVoltage(double voltage) {
    m_coralLeft.setVoltage(voltage);
    m_coralRight.setVoltage(voltage);
  }

  private void setIntake() {
    setVoltage(CoralConstants.intakeVoltage);
  }

  private void setL2Score() {
    m_logger.trace("Setting L2 eject");
    setVoltage(CoralConstants.l2EjectVoltage);
  }

  private void setL3Score() {
    m_logger.trace("Setting L3 eject");
    setVoltage(CoralConstants.l3EjectVoltage);
  }

  private void setL4Score() {
    m_logger.trace("Setting L4 eject");
    setVoltage(CoralConstants.l4EjectVoltage);
  }

  private void setL1Score() {
    m_coralLeft.setVoltage(CoralConstants.l1LeftEjectVoltage);
    m_coralRight.setVoltage(CoralConstants.l1RightEjectVoltage);
  }

  private boolean getFrontCANrange() {
    return m_frontRange.getIsDetected().getValue();
  }

  private boolean getUpperCANrange() {
    return m_upperRange.getIsDetected().getValue();
  }

  private boolean getInnerCANrange() {
    return m_innerRange.getIsDetected().getValue();
  }

  public Trigger holding() {
    return new Trigger(() -> {
      if (Robot.isReal()) {
        boolean holding = getFrontCANrange() && !getUpperCANrange();
        m_logger.trace("holding: {}", holding);
        return holding;
      } else {
        boolean present = SmartDashboard.getBoolean("Coral present", false);
        SmartDashboard.putBoolean("Coral present", present);
        return present;
      }
    });
  }

  private void stop() {
    setVoltage(0);
  }

  public Trigger present() {
    return new Trigger(() -> getUpperCANrange() || getInnerCANrange() || getFrontCANrange());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Inner CANrange", getInnerCANrange());
    SmartDashboard.putBoolean("Coral CANrange", getFrontCANrange());
    SmartDashboard.putBoolean("OCS", getUpperCANrange());
    SmartDashboard.putBoolean("HAS CORAL", holding().getAsBoolean());

    if (m_voltageChanged) {
      m_coralLeft.setVoltage(m_voltage);
      m_coralRight.setVoltage(m_voltage);
      m_voltageChanged = false;
    }
  }

  protected void passive() {}

  /**
   * Intakes a game piece. The command ends when the piece is fully in the robot.
   */
  public Command intake() {
    return Commands.sequence(
        runOnce(this::setIntake),
        Commands.waitUntil(holding()))

        .finallyDo(this::stop);
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

}
