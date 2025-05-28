package frc.robot.binding;

import edu.wpi.first.wpilibj.PS5Controller.Button;

public class BindingConstants {
  public static final int kDriverPort = 0;
  public static final int operatorPort = 1;

  public static class Driver {
    public static final int xAxis = 1;
    public static final int yAxis = 0;
    public static final int rotAxis = 3;

    public static final boolean kFlipX = false;
    public static final boolean kFlipY = true;
    public static final boolean kFlipRot = false;

    public static final int kResetHeading = 1;
    public static final int kSmartAlign = 2;
    public static final int kRightAlign = 3;
    public static final int kLeftAlign = 4;

    public static final double deadband = 0.01;
  }

  public static class Operator {
    public static final int kL1 = 180;
    public static final int kL2 = 270;
    public static final int kL3 = 90;
    public static final int kL4 = 0;

    public static final int kSecondaryL1 = 14;

    public static final int kEjectCoral = Button.kL2.value;

    public static final int kLeftAlign = Button.kSquare.value;
    public static final int kRightAlign = Button.kCircle.value;

    public static final int kLowerAlgae = Button.kCross.value;
    public static final int kUpperAlgae = Button.kTriangle.value;
    public static final int kGroundAlgaeIntake = 180;
    public static final int kProcessor = 90;
    public static final int kHighGroundAlgaeIntake = 270;
    public static final int kNet = 0;
    public static final int kAlgae = Button.kR2.value;

    public static final int kRaiseClimb = Button.kCreate.value;
    public static final int kClimb = Button.kOptions.value;

    public static final int kStow = Button.kPS.value;

    public static final int kCoralIntake = Button.kL1.value;

    public static final int kCalibrateElevator = 15;

    public static final int kRightFunnel = 11;
    public static final int kLeftFunnel = 12;
  }
}

