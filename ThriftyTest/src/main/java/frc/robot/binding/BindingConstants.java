package frc.robot.binding;

import edu.wpi.first.wpilibj.PS5Controller.Button;

public class BindingConstants {
  public static final int driverPort = 0;
  public static final int operatorPort = 1;

  public static class Driver {
    public static final int xAxis = 1;
    public static final int yAxis = 0;
    public static final int rotAxis = 3;

    public static final boolean flipX = false;
    public static final boolean flipY = true;
    public static final boolean flipRot = false;

    public static final int resetHeading = 1;
    public static final int processor = 2;

    public static final double deadband = 0.01;
  }

  public static class Operator {
    public static final int L1 = 180;
    public static final int L2 = 270;
    public static final int L3 = 90;
    public static final int L4 = 0;

    public static final int secondaryL1 = 14;

    public static final int ejectCoral = Button.kL2.value;

    public static final int leftReef = Button.kSquare.value;
    public static final int rightReef = Button.kCircle.value;

    public static final int lowAlgae = Button.kCross.value;
    public static final int highAlgae = Button.kTriangle.value;
    public static final int ground = 180;
    public static final int processor = 90;
    public static final int highGround = 270;
    public static final int net = 0;
    public static final int algaeModeButton = Button.kR2.value;

    public static final int autoProcessor = Button.kR1.value;

    public static final int climbUp = Button.kCreate.value;
    public static final int climb = Button.kOptions.value;

    public static final int stow = Button.kPS.value;

    public static final int intake = Button.kL1.value;

    public static final int zeroElevator = 15;

    public static final int rightFunnel = 11;
    public static final int leftFunnel = 12;
  }
}

