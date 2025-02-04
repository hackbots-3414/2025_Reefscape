// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdle.LEDStripType;


// /** Add your docs here. */
// public class LedSubsystem {
//     private static double matchTime = 0;
//     private int r = 0;
//     private int g = 0;
//     private int b = 0;

//     CANdle
//}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;
import frc.robot.Constants.LedConstants;
import frc.robot.RobotContainer.JoystickChoice;

public class LedSubsystem extends SubsystemBase {
  // final static Logger logger = LoggerFactory.getLogger(LedSubsystem.class);
  // private static final double IN_RANGE = 0.87; // BLUE
  // private static final double NOTE_ONBOARD = 0.77; // GREEN
  // private static final double END_GAME_15 = -0.61; // RED
  // private static final double END_GAME_30 = -0.93; // WHITE
  // private static final double AIM_READY = 0.69; // YELLOW
  private static double matchTime = 0;

  // private static final double NOTE_IN_VIEW = 0.65;
  /// private static final double CLIMBER_ACTIVATED = -0.95;
  // private static final double DEFAULT = 0.91; // PURPPLE
  // private static final FireAnimation HACKBOTS_FIRE = new FireAnimation(1.0,
  // 0.1, LedConstants.nbrLED, 0.2, 0.2);
  // private static final StrobeAnimation HACKBOTS_STROBE = new
  // StrobeAnimation(255, 255, 255, 0,
  // LEDConstants.strobeSpeed,
  // LEDConstants.nbrLED);
  // private static final SingleFadeAnimation GREEN_FLASH_ANIMATION_LEFT = new
  // SingleFadeAnimation(0, 255, 0, 0,
  // LEDConstants.flashSpeed, LEDConstants.leftNumLED, LEDConstants.leftOffset);
  // private static final SingleFadeAnimation GREEN_FLASH_ANIMATION_RIGHT = new
  // SingleFadeAnimation(0, 255, 0, 0,
  // LEDConstants.flashSpeed, LEDConstants.rightNumLED, LEDConstants.rightOffset);
  // private static final SingleFadeAnimation GREEN_FLASH_ANIMATION_TOP = new
  // SingleFadeAnimation(0, 255, 0, 0,
  // LEDConstants.flashSpeed, LEDConstants.topNumLED, LEDConstants.topOffset);
  // private static final SingleFadeAnimation YELLOW_FLASH_ANIMATION_LEFT = new
  // SingleFadeAnimation(250, 120, 0, 0,
  // LEDConstants.flashSpeed, LEDConstants.leftNumLED, LEDConstants.leftOffset);
  // private static final SingleFadeAnimation YELLOW_FLASH_ANIMATION_RIGHT = new
  // SingleFadeAnimation(250, 120, 0, 0,
  // LEDConstants.flashSpeed, LEDConstants.rightNumLED, LEDConstants.rightOffset);
  // private static final SingleFadeAnimation YELLOW_FLASH_ANIMATION_TOP = new
  // SingleFadeAnimation(250, 120, 0, 0,
  // LEDConstants.flashSpeed, LEDConstants.topNumLED, LEDConstants.topOffset);
  // private static final StrobeAnimation RED_STROBE_ANIMATION_LEFT = new
  // StrobeAnimation(255, 0, 0, 0,
  // LEDConstants.strobeSpeed, LEDConstants.leftNumLED, LEDConstants.leftOffset);
  // private static final StrobeAnimation RED_STROBE_ANIMATION_RIGHT = new
  // StrobeAnimation(255, 0, 0, 0,
  // LEDConstants.strobeSpeed, LEDConstants.rightNumLED,
  // LEDConstants.rightOffset);
  // private static final TwinkleAnimation TWINKLE_ANIMATION = new
  // TwinkleAnimation(255, 0, 255, 0, 0.5,
  // LEDConstants.nbrLED, TwinklePercent.Percent42);
  // private static final LarsonAnimation LARSON_ANIMATION = new
  // LarsonAnimation(255, 0, 255, 0, 0.1, LEDConstants.nbrLED,
  // LarsonAnimation.BounceMode.Back, 1);
  private Supplier<Boolean> isInRange, noteInView, isInFeedRange;
  private boolean noteOnBoard = false;
  private boolean noteOnBoardTest = false;
  private boolean isInRangeTest = false;
  private boolean noteInViewTest = false;
  private int r = 0;
  private int g = 0;
  private int b = 0;
  private int offsetLED = 0;
  private int nbrLED = 0;
  private int ledStripEndIndex = 0;
  private int ledStripStartIndex = 0;
  private boolean endgameWarningStarted = false;
  private boolean endgameAlertStarted = false;
  private boolean inAuton = false;
  private boolean inTeleop = false;
  private Transport transport;
  private Intake intake;

  // private static final String[] LABELS = { "In Range", "Note Onboard", "End
  // Game 15", "End Game 30", "Target Locked" };

  private static enum LED_MODE {
    IN_RANGE, IN_FEED_RANGE, NOTE_ONBOARD, END_GAME_WARNING, END_GAME_ALERT, ALIGNED, DEFAULT, NOTE_IN_VIEW, INTAKE, BADCONTROLLER;
  };

  private static LED_MODE chosenMode = null;

  CANdle ledcontroller = new CANdle(LEDConstants.candleCanid);
  // private int currentMode = 0;

  public LedSubsystem(Transport transport, Intake intake, Supplier<Boolean> isInRange, Supplier<Boolean> isInFeedRange, Supplier<Boolean> noteInView) {
    this.transport = transport;
    this.intake = intake;
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.7; // dim the LEDs to 70% brightness
    ledcontroller.configAllSettings(config);
    defaultColors();

    // uncomment to test LED strips
    // ledcontroller.setLEDs(255, 0, 0, 0, LEDConstants.leftOffset,
    // LEDConstants.leftNumLED);
    // ledcontroller.setLEDs(0, 0, 255, 0, LEDConstants.topOffset,
    // LEDConstants.topNumLED);
    // ledcontroller.setLEDs(0,255, 0, 0, LEDConstants.insideOffset,
    // LEDConstants.insideNumLED);
    // ledcontroller.setLEDs(255, 0,255, 0, LEDConstants.rightOffset,
    // LEDConstants.rightNumLED);
    // end of test pattern

    // this.noteOnBoard = noteOnBoard;
    this.isInRange = isInRange;
    this.isInFeedRange = isInFeedRange;
    this.noteInView = noteInView;
    // Hackbot Purple Code : [0x67, 0x2C, 0x91]
    // #672C91
    SmartDashboard.putBoolean("noteOnBoardTest", noteOnBoardTest);
    SmartDashboard.putBoolean("IsInRangeTest", isInRangeTest);
    SmartDashboard.putBoolean("noteInViewTest", noteInViewTest);
    // SmartDashboard.putNumber("r new", r);
    // SmartDashboard.putNumber("b new", b);
    // SmartDashboard.putNumber("g new", g);
    // SmartDashboard.putNumber("r", r);
    // SmartDashboard.putNumber("g", g);
    // SmartDashboard.putNumber("b", b);
  }

  // private void resetStatus() {
  // for (String element : LABELS) {
  // SmartDashboard.putBoolean(element, false);
  // }
  // }
  
  @Override
  public void periodic() {
    // noteOnBoardTest = SmartDashboard.getBoolean("noteOnBoardTest", noteOnBoardTest);
    // isInRangeTest = SmartDashboard.getBoolean("IsInRangeTest", isInRangeTest);
    // noteInViewTest = SmartDashboard.getBoolean("noteInViewTest", noteInViewTest);
    // r = (int)SmartDashboard.getNumber("r", r);
    // g = (int)SmartDashboard.getNumber("g", g);
    // b = (int)SmartDashboard.getNumber("b", b);
    // ledcontroller.setLEDs(r, g, b);

    noteOnBoard = transport.getNoteOnBoard();
    matchTime = DriverStation.getMatchTime();
    // if (inTeleop == false && matchTime > 60) {
    // inTeleop = true;
    // }

    inAuton = DriverStation.isAutonomousEnabled();
    inTeleop = DriverStation.isTeleopEnabled();
    // if (inTeleop == false && endgameWarningStarted == false && matchTime > 0) {
    // setColor("DEFAULT", 0, 2, "SOLID");
    // }
    // SmartDashboard.putBoolean("In Auton", inAuton);
    // SmartDashboard.putBoolean("In Teleop", inTeleop);
    SmartDashboard.putNumber("Match Time", matchTime);
    // SmartDashboard.putBoolean("badController", badController());
    if (inTeleop || inAuton) {
      if (matchTime <= LEDConstants.endgameWarning && matchTime > 0 && !inAuton) {
        ledStripEndIndex = 0;
        ledStripStartIndex = 0;

        if (matchTime > LEDConstants.endgameAlert && endgameWarningStarted == false && !inAuton) {
          endgameWarningStarted = true;
          setColor("RED", 1, 2, "SOLID");
        }

        else if (matchTime <= LEDConstants.endgameAlert && endgameAlertStarted == false && !inAuton && matchTime > 0) {
          endgameAlertStarted = true;
          setColor("RED", 1, 2, "STROBE");
        }
      } else {
        ledStripStartIndex = 0;
        ledStripEndIndex = 2;
      }
      // / Note In View: Yellow
      // Note on Board : Green Medium Flash (During END Game Make sure this Overides
      // the End Game Alert or Warning 3 sec timer)
      // End Game Warning (20): White
      // End Game Alert (10): Strobe White
      // Shoot Alignment Happening : Slow Blue
      // When Aligned: Stop Strobe: Fast Flash Blue
      // InRange for shooting: Blue


      if (noteOnBoard && isInRange.get()) {
        // noteOnboardTest should be noteOnBoard.get()
        // Do this for all test Variables
        if (chosenMode != LED_MODE.IN_RANGE) {
          chosenMode = LED_MODE.IN_RANGE;
          setColor("BLUE", ledStripStartIndex, ledStripEndIndex, "SOLID");
        }
      } else if (noteOnBoard && isInFeedRange.get()) {
        if (chosenMode != LED_MODE.IN_FEED_RANGE) {
          chosenMode = LED_MODE.IN_FEED_RANGE;
          setColor("ORANGE", ledStripStartIndex, ledStripEndIndex, "SOLID");
        }
      } else if (noteOnBoard) {
        if (chosenMode != LED_MODE.NOTE_ONBOARD) {
          chosenMode = LED_MODE.NOTE_ONBOARD;
          setColor("GREEN", ledStripStartIndex, ledStripEndIndex, "STROBE");
        }
        } else if (intake.getIntakeIR() == true) {
        if (chosenMode != LED_MODE.INTAKE) {
          chosenMode = LED_MODE.INTAKE;
          setColor("GREEN", ledStripStartIndex, ledStripEndIndex, "FLASH");
        }
      } else if (noteInView.get()) {
        if (chosenMode != LED_MODE.NOTE_IN_VIEW) {
          chosenMode = LED_MODE.NOTE_IN_VIEW;
          setColor("YELLOW", ledStripStartIndex, ledStripEndIndex, "FLASH");

        }
      }

      else {
        if (chosenMode != LED_MODE.DEFAULT) {
          chosenMode = LED_MODE.DEFAULT;
          setColor("DEFAULT", ledStripStartIndex, ledStripEndIndex, "SOLID");

        }

      }
    } else {
      if (badController()) {
        if (chosenMode != LED_MODE.BADCONTROLLER) {
          chosenMode = LED_MODE.BADCONTROLLER;
          setColor("RED", 0, 2, "STROBE");
        } 
      } else {
         if (chosenMode != LED_MODE.DEFAULT) {
          chosenMode = LED_MODE.DEFAULT;
          defaultColors();

        }
      }
    }

  }
  private void defaultColors() {
    ledcontroller.clearAnimation(0);
    ledcontroller.clearAnimation(1);
    ledcontroller.clearAnimation(2);
    ledcontroller.clearAnimation(3);
    ledcontroller.animate(
        new LarsonAnimation(255, 0, 255, 0, 0.75, LEDConstants.numLED, LarsonAnimation.BounceMode.Back, 14), 0);
    ledcontroller.animate(
        new LarsonAnimation(255, 0, 255, 0, 0.50, LEDConstants.numLED, LarsonAnimation.BounceMode.Back, 7), 1);
    setColor("DEFAULT", 3, 3, "FLASH");
  }
  private boolean badController() {
    if (!DriverStation.isJoystickConnected(0) || !DriverStation.isJoystickConnected(1)) {
      return true;
    }

    String joystick1Name = DriverStation.getJoystickName(1).toLowerCase();

    return !DriverStation.getJoystickName(0).contains("InterLinkDX") &&
        !((DriverConstants.operatorController == JoystickChoice.XBOX &&
            (joystick1Name.contains("xbox")) ||
            (joystick1Name.contains("gamepad"))) ||
            (DriverConstants.operatorController == JoystickChoice.PS5 &&
                joystick1Name.contains("dualsense")));
  }

  public void setColor(String color, int LedStripStart, int LedStripEnd, String pattern) {
    if (color == "BLUE") {
      r = 0;
      g = 0;
      b = 255;
    } else if (color == "GREEN") {
      r = 0;
      g = 255;
      b = 0;

    } else if (color == "RED") {
      r = 255;
      g = 0;
      b = 0;

    } else if (color == "YELLOW") {
      r = 255;
      g = 120;
      b = 0;
    } else if (color == "ORANGE") {
      r = 255;
      g = 85;
      b = 0;
    } else {
      r = 255;
      g = 0;
      b = 255;

    }

    for (int x = LedStripStart; x <= LedStripEnd; x++) {

      if (x == 0) {
        offsetLED = LEDConstants.topOffset;
        nbrLED = LEDConstants.topNumLED;

      } else if (x == 1) {
        offsetLED = LEDConstants.leftOffset;
        nbrLED = LEDConstants.leftNumLED;
      } else if (x == 2) {
        offsetLED = LEDConstants.rightOffset;
        nbrLED = LEDConstants.rightNumLED;
      } else {
        offsetLED = LEDConstants.insideOffset;
        nbrLED = LEDConstants.insideNumLED;
      }

      ledcontroller.clearAnimation(x);
      // SmartDashboard.putNumber("r new", r);
      // SmartDashboard.putNumber("b new", b);
      // SmartDashboard.putNumber("g new", g)

      if (pattern == "SOLID") {
        ledcontroller.setLEDs(r, g, b, 0, offsetLED, nbrLED);
      } else if (pattern == "FLASH") {
        ledcontroller.animate(new SingleFadeAnimation(r, g, b, 0, LEDConstants.flashSpeed, nbrLED, offsetLED), x);
      } else if (pattern == "STROBE") {
        ledcontroller.animate(new StrobeAnimation(r, g, b, 0, LEDConstants.strobeSpeed, nbrLED, offsetLED), x);
      } else if (pattern == "TWINKLE") {
        ledcontroller.animate(new TwinkleAnimation(r, g, b, 0, 0.5, nbrLED, TwinklePercent.Percent42, offsetLED), x);
      } else { // LARSON
        ledcontroller.animate(
            new LarsonAnimation(r, g, b, 0, 0.75, LEDConstants.numLED, LarsonAnimation.BounceMode.Back, 7), x);

      }

    }
  }
}
