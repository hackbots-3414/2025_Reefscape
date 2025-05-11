// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SimConstants;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import frc.robot.subsystems.PassiveSubsystem;

public class Pivot extends PassiveSubsystem {
  private final TalonFX m_pivot = new TalonFX(PivotConstants.kMotorID);

  private double m_position;

  private double m_reference;

  private SingleJointedArmSim m_armSim;
  private final DCMotor m_gearbox = DCMotor.getKrakenX60(1); // 2 motors (left and right)

  private Mechanism2d m_mechVisual;
  private MechanismRoot2d m_mechRoot;
  private MechanismLigament2d m_armLigament;

  private double m_speed;
  private boolean m_speedChanged;

  public Pivot() {
    super();
    configSim();
    configMotor();
  }

  private void configMotor() {
    m_pivot.getConfigurator().apply(PivotConstants.kMotorConfig, 0.2);
    m_pivot.setPosition(PivotConstants.kRotorOffset);
  }

  private void configSim() {
    m_armSim = new SingleJointedArmSim(
        PivotConstants.kPlant,
        m_gearbox,
        PivotConstants.kGearRatio,
        PivotConstants.kArmLength,
        PivotConstants.kRadiansAtZero,
        PivotConstants.kRadiansAtMax,
        true, // Add noise for realism
        PivotState.Stow.position() // Starting angle
    );

    m_mechVisual = new Mechanism2d(1.0, 1.0); // Width/height in meters
    m_mechRoot = m_mechVisual.getRoot("ArmRoot", 0.5, 0.0); // Center at (0.5, 0)
    m_armLigament = m_mechRoot
        .append(
            new MechanismLigament2d("Arm", PivotConstants.kArmLength, Math.toDegrees(m_position)));
    SmartDashboard.putData("Pivot/Visualization", m_mechVisual);
  }

  MotionMagicVoltage control = new MotionMagicVoltage(0);

  private void setPosition(double goal) {
    take();
    if (RobotObserver.getAlgaePieceHeld()) {
      m_pivot.setControl(control.withPosition(goal).withSlot(1));
    } else {
      m_pivot.setControl(control.withPosition(goal).withSlot(0));
    }
    m_reference = goal;
  }

  private void setPosition(PivotState state) {
    setPosition(state.position());
  }

  public double getPosition() {
    return m_position;
  }

  public double getReference() {
    return m_reference;
  }

  public Trigger ready() {
    return new Trigger(() -> Math.abs(getReference() - getPosition()) < PivotConstants.kTolerance);
  }

  private double getPositionUncached() {
    if (Robot.isReal()) {
      return m_pivot.getPosition().getValueAsDouble();
    } else {
      return m_armSim.getAngleRads();
    }
  }

  @Override
  public void periodic() {
    m_armLigament.setAngle(Math.toDegrees(m_position));

    m_position = getPositionUncached();

    if (m_speedChanged) {
      m_pivot.setControl(new DutyCycleOut(m_speed));
      m_speedChanged = false;
    }

    SmartDashboard.putBoolean("Pivot/Ready", ready().getAsBoolean());
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation with the motor voltage
    double appliedVolts = m_pivot.get() * RobotController.getBatteryVoltage();
    m_armSim.setInput(appliedVolts);
    m_armSim.update(SimConstants.k_simPeriodic);

    // Simulate battery voltage
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
  }

  protected void passive() {
    setPosition(PivotState.Stow);
  }

  /**
   * Drives the pivot to the given PivotState This command ends when the state is reached.
   */
  public Command go(PivotState state) {
    return Commands.sequence(
        runOnce(() -> setPosition(state)),
        Commands.waitUntil(ready()));
  }

}
