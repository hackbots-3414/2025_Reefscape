// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.LoopTimer;
import frc.robot.utils.OnboardLogger;
import frc.robot.utils.StatusSignalUtil;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final LoopTimer m_loopTimer;

  private final OnboardLogger m_ologger;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_loopTimer = new LoopTimer("Robot");
    m_ologger = new OnboardLogger("Robot");
    m_ologger.registerDouble("Battery Voltage", RobotController::getBatteryVoltage);
  }

  @Override
  public void robotInit() {
    RobotObserver.setField(new Field2d());
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    m_robotContainer.enablePDPSwitch();
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    LiveWindow.disableAllTelemetry();
  }

  @Override
  public void robotPeriodic() {
    m_loopTimer.reset();
    CommandScheduler.getInstance().run();
    StatusSignalUtil.refreshAll();
    SmartDashboard.putNumber("Robot/Match Time", DriverStation.getMatchTime());
    m_ologger.log();
    m_loopTimer.log();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
