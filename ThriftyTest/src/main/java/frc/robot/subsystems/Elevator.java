// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public Elevator() {}

  public void setLevel(int level) {
    switch (level) {
      case 1 -> setL1();
      case 2 -> setL2();
      case 3 -> setL3();
      case 4 -> setL4();
      case 0 -> setStow();
      default -> setStow();
    }
  }

  public void setStow() {}
  public void setL1() {}
  public void setL2() {}
  public void setL3() {}
  public void setL4() {}

  public boolean atSetpoint() {
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
