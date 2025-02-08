// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.logging.Logger;

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
import frc.robot.RobotContainer.JoystickChoice;
import frc.robot.Constants.DriverConstants;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;
import frc.robot.Constants.LEDConstants;
public class LedSubsystem extends SubsystemBase {
  private static double matchTime = 0;
  // private Supplier<Boolean>  coralInView;
  private boolean coralOnBoard = false;
  // private boolean coralOnBoardTest = false;
  // private boolean isInRangeTest = false;
  private boolean coralInViewTest = false;
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
  private static enum LED_MODE {
    CORAL_ONBOARD, END_GAME_WARNING, END_GAME_ALERT, DEFAULT, CORAL_IN_VIEW,
    BADCONTROLLER;
  };

  private static LED_MODE chosenMode = null;

  CANdle ledcontroller = new CANdle(LEDConstants.candleCanid);
  // private int currentMode = 0;

  public LedSubsystem() {
  SmartDashboard.putBoolean("coralInView", true);

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.7; // dim the LEDs to 70% brightness
    ledcontroller.configAllSettings(config);
    defaultColors();

    // uncomment to test LED strips
    ledcontroller.setLEDs(255, 0, 0, 0, LEDConstants.leftOffset,
        LEDConstants.leftNumLED);
    ledcontroller.setLEDs(0, 0, 255, 0, LEDConstants.topOffset,
        LEDConstants.topNumLED);
    ledcontroller.setLEDs(0, 255, 0, 0, LEDConstants.insideOffset,
        LEDConstants.insideNumLED);
    ledcontroller.setLEDs(255, 0, 255, 0, LEDConstants.rightOffset,
        LEDConstants.rightNumLED);
    // end of test pattern

    // Hackbot Purple Code : [0x67, 0x2C, 0x91]
    // #672C91

    // this.isInRange = isInRange;
    // this.coralInView = coralInView;
  }

  @Override
  public void periodic() {

    matchTime = DriverStation.getMatchTime();

    inAuton = DriverStation.isAutonomousEnabled();
    inTeleop = DriverStation.isTeleopEnabled();
    // if (inTeleop == false && endgameWarningStarted == false && matchTime > 0) {
    // setColor("DEFAULT", 0, 2, "SOLID");
    // }
    coralInViewTest = SmartDashboard.getBoolean("coralInView", true);

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
    }

    if (coralOnBoard) {
      if (chosenMode != LED_MODE.CORAL_ONBOARD) {
        chosenMode = LED_MODE.CORAL_ONBOARD;
        setColor("GREEN", ledStripStartIndex, ledStripEndIndex, "STROBE");
      }
      // } else if (intake.getIntakeIR() == true) {
      // if (chosenMode != LED_MODE.INTAKE) {
      // chosenMode = LED_MODE.INTAKE;
      // setColor("GREEN", ledStripStartIndex, ledStripEndIndex, "FLASH");
      // }
      else if (coralInViewTest) {
        if (chosenMode != LED_MODE.CORAL_IN_VIEW) {
          chosenMode = LED_MODE.CORAL_IN_VIEW;
          setColor("YELLOW", ledStripStartIndex, ledStripEndIndex, "FLASH");

        }
      } else {
        if (chosenMode != LED_MODE.DEFAULT) {
          chosenMode = LED_MODE.DEFAULT;
          setColor("DEFAULT", ledStripStartIndex, ledStripEndIndex, "SOLID");

        }
    }
  }    
        else {
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

      b = 0;
    } else {
      r = 255;
      //
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
