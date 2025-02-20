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
import frc.robot.Constants.LEDConstants;
import frc.robot.RobotContainer.JoystickChoice;

public class LedSubsystem extends SubsystemBase {
  private static double matchTime = 0;
  // boolean coralOnBoardTest = false;
  // private Supplier<Boolean> isInRange;
  // private boolean coralOnBoard = false;
  private boolean coralOnBoardTest = false;
  private boolean isInRangeTest = false;
  private int r = 0;
  private int g = 0;
  private int b = 0;
  private int offsetLED = 0;
  private int nbrLED = 0;
  private int ledStripEndIndex = 12;
  private int ledStripStartIndex = 0;
  private boolean inAuton = false;
  private boolean inTeleop = false;
  private Coral coral;

  private static enum LED_MODE {
    CORAL_ONBOARD, END_GAME_WARNING, END_GAME_ALERT, DEFAULT,
    BADCONTROLLER, CORAL_INTAKE, ALIGNED;
  };

  private static enum LED_COLOR {
    RED, YELLOW, GREEN, PURPLE, BLUE;
  };

  private static enum LED_PATTERN {
    TWINKLE, STROBE, LARSON, FLASH, SOLID, NOCHANGE;
  };

  private static LED_MODE chosenMode = null;

  CANdle ledcontroller = new CANdle(LEDConstants.candleCanid);
  // private int currentMode = 0;

  public LedSubsystem(Coral coral) {
    this.coral = coral;
    SmartDashboard.putBoolean("coralOnBoardTest", true);
    SmartDashboard.putBoolean("isInRangeTest", true);

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.7; // dim the LEDs to 70% brightness
    ledcontroller.configAllSettings(config);
    defaultColors();

    // uncomment to test LED strips
    // ledcontroller.setLEDs(255, 0, 0, 0, LEDConstants.funnelOffset,
    // LEDConstants.funnelNumLED);
    // ledcontroller.setLEDs(255, 0, 255, 0, LEDConstants.elevatorOffset,
    // LEDConstants.elevatorNumLED);
    // // end of test pattern

    // Hackbot Purple Code : [0x67, 0x2C, 0x91]
    // #672C91

    // this.isInRange = isInRange;
    // this.coralInView = coralInView;
  }

  @Override
  public void periodic() {

    // coralOnBoard = coral.holdingPiece();
    matchTime = DriverStation.getMatchTime();
    inAuton = DriverStation.isAutonomousEnabled();
    inTeleop = DriverStation.isTeleopEnabled();
    // if (inTeleop == false && endgameWarningStarted == false && matchTime > 0) {
    // setColor("DEFAULT", 0, 2, "SOLID");
    // }
    coralOnBoardTest = SmartDashboard.getBoolean("coralOnBoardTest", true);
    System.out.println(coralOnBoardTest);
    isInRangeTest = SmartDashboard.getBoolean("isInRangeTest", true);
    System.out.println(isInRangeTest);

    SmartDashboard.putBoolean("Bad Controller", badController());
    // SmartDashboard.putBoolean("In Auton", inAuton);
    // SmartDashboard.putBoolean("In Teleop", inTeleop);
    SmartDashboard.putNumber("Match Time", matchTime);
    // SmartDashboard.putBoolean("badController", badController());
    if (badController()) {
      if (chosenMode != LED_MODE.BADCONTROLLER) {
        chosenMode = LED_MODE.BADCONTROLLER;
        setColor(LED_COLOR.RED, 0, 2, LED_PATTERN.STROBE);
      }
    } else if (inTeleop || inAuton) {
      if (inTeleop) {
        if (matchTime <= LEDConstants.endgameWarning && matchTime > 0) {
          ledStripEndIndex = 0;
          ledStripStartIndex = 0;
          if (matchTime <= LEDConstants.endgameAlert && matchTime > 0) {
            if (chosenMode != LED_MODE.END_GAME_ALERT){
              chosenMode = LED_MODE.END_GAME_ALERT;
            setColor(LED_COLOR.YELLOW, 0, 1, LED_PATTERN.STROBE); // Changed
            }
          } else if (matchTime <= LEDConstants.endgameWarning) {
            if(chosenMode != LED_MODE.END_GAME_WARNING){
              chosenMode = LED_MODE.END_GAME_WARNING;
              setColor(LED_COLOR.YELLOW, 0, 1, LED_PATTERN.SOLID); // Changed             
            }
          }
        } else if (coralOnBoardTest && isInRangeTest) {
          if (chosenMode != LED_MODE.CORAL_ONBOARD) {
            chosenMode = LED_MODE.CORAL_ONBOARD;
            setColor(LED_COLOR.BLUE, ledStripStartIndex, ledStripEndIndex, LED_PATTERN.STROBE);
          }
        } else if (coralOnBoardTest) {
          if (chosenMode != LED_MODE.CORAL_ONBOARD) {
            chosenMode = LED_MODE.CORAL_ONBOARD;
            setColor(LED_COLOR.BLUE, ledStripStartIndex, ledStripEndIndex, LED_PATTERN.SOLID);
          }
        } else if (chosenMode != LED_MODE.DEFAULT) {
          chosenMode = LED_MODE.DEFAULT;
          setColor(LED_COLOR.PURPLE, ledStripStartIndex, ledStripEndIndex, LED_PATTERN.NOCHANGE);
        }
      }
    } else {
      ledStripStartIndex = 0;
      ledStripEndIndex = 2;
    }
  }
  // } else if (coralOnBoardTest && isInRangeTest) {
  // if (chosenMode != LED_MODE.CORAL_ONBOARD) {
  // chosenMode = LED_MODE.CORAL_ONBOARD;
  // setColor(LED_COLOR.BLUE, ledStripStartIndex, ledStripEndIndex,
  // LED_PATTERN.STROBE);
  // }
  // } else if (coralOnBoardTest) {
  // if (chosenMode != LED_MODE.CORAL_ONBOARD) {
  // chosenMode = LED_MODE.CORAL_ONBOARD;
  // setColor(LED_COLOR.BLUE, ledStripStartIndex, ledStripEndIndex,
  // LED_PATTERN.SOLID);
  // }
  // } else if (chosenMode != LED_MODE.DEFAULT) {
  // chosenMode = LED_MODE.DEFAULT;
  // setColor(LED_COLOR.PURPLE, ledStripStartIndex, ledStripEndIndex,
  // LED_PATTERN.NOCHANGE);
  // }

  private void defaultColors() {
    ledcontroller.clearAnimation(0);
    ledcontroller.clearAnimation(1);
    ledcontroller.clearAnimation(2);
    ledcontroller.clearAnimation(3);
    setColor(LED_COLOR.PURPLE, 1, 1, LED_PATTERN.FLASH);
    ledcontroller.animate(
        new LarsonAnimation(255, 0, 255, 0, 0.75, LEDConstants.numLED, LarsonAnimation.BounceMode.Back, 14), 0);
    ledcontroller.animate(
        new LarsonAnimation(255, 0, 255, 0, 0.50, LEDConstants.numLED, LarsonAnimation.BounceMode.Back, 7), 1);
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

  public void setColor(LED_COLOR color, int LedStripStart, int LedStripEnd, LED_PATTERN pattern) {
    // ledcontroller.clearAnimation(0);
    // ledcontroller.clearAnimation(1);

    switch (color) {
      case BLUE:
        r = 0;
        g = 0;
        b = 255;
        break;

      case GREEN:
        r = 0;
        g = 255;
        b = 0;
        break;

      case RED:
        r = 255;
        g = 0;
        b = 0;
        break;

      case YELLOW:
        r = 255;
        g = 120;
        b = 0;
        break;

      case PURPLE:
        r = 255;
        g = 0;
        b = 255;
        break;
    }

    for (int x = LedStripStart; x <= LedStripEnd; x++) {

      switch (x) {
        case 0:
          offsetLED = LEDConstants.funnelOffset;
          nbrLED = LEDConstants.funnelNumLED;
          break;
        default:
          offsetLED = LEDConstants.elevatorOffset;
          nbrLED = LEDConstants.elevatorNumLED;
          break;
      }

      switch (pattern) {
        case SOLID:
          ledcontroller.clearAnimation(0);
          ledcontroller.clearAnimation(1);
          ledcontroller.setLEDs(r, g, b, 0, offsetLED, nbrLED);
          break;
        case FLASH:
          ledcontroller.animate(new SingleFadeAnimation(r, g, b, 0, LEDConstants.flashSpeed, nbrLED, offsetLED), x);
          break;
        case STROBE:
          ledcontroller.animate(new StrobeAnimation(r, g, b, 0, LEDConstants.strobeSpeed, nbrLED, offsetLED), x);
          break;
        case TWINKLE:
          ledcontroller.animate(new TwinkleAnimation(r, g, b, 0, 0.5, nbrLED, TwinklePercent.Percent42, offsetLED), x);
          break;
        case NOCHANGE:
          break;
        case LARSON:
          ledcontroller.animate(
              new LarsonAnimation(r, g, b, 0, 0.75, LEDConstants.numLED, LarsonAnimation.BounceMode.Back, 7), x);
      }

    }
  }
}
