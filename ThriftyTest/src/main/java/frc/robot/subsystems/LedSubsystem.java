// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.io.Console;
import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;
import frc.robot.Constants.LEDConstants;
import frc.robot.RobotContainer.JoystickChoice;
import frc.robot.generated.TunerConstants;

public class LedSubsystem extends SubsystemBase {
  private static double matchTime = 0;
  // private Supplier<Boolean> isInRange;
  // private boolean coralOnBoard = false;
  // private boolean algaeOnBoard = false;
  private boolean climbed = false;
  private boolean coralOnBoardTest = false;
  private boolean isInRangeTest = false;
  private boolean climbedTest = false;
  private int r = 0;
  private int g = 0;
  private int b = 0;
  private int offsetLED = 0;
  private int nbrLED = 0;
  private boolean inAuton = false;
  private boolean inTeleop = false;
  // private boolean climbedTest = false;
  private Coral coral;
  // private AlgaeRoller algae;  - Only If Algae is necessary
  private int selectedSlot = 0;
  private boolean initialClearRan = false;

  /*
   * After Merge What needs to happen:
   * - Make isInRange a Supplier, and put into the Constructor (Ledsubsystem)
   * - Make sure to get CoralOnboard from the Coral Subsystem,
   * - Lastly Make sure that everything is working with Main code
   */

  private static enum LED_MODE {
    CORAL_ON_BOARD, CORAL_READY, END_GAME_WARNING, END_GAME_ALERT, DEFAULT,
    BADCONTROLLER, IN_RANGE, CLIMBED, ALGAE_ON_BOARD;
  };

  private static enum LED_COLOR {
    RED, YELLOW, GREEN, PURPLE, BLUE, WHITE, OFF;
  };

  private static enum LED_PATTERN {
    TWINKLE, STROBE, LARSON, FLASH, SOLID, CLEAR, RAINBOW;
  };

  private static enum LED_SECTION {
    FUNNEL, ELEVATOR;
  }

  private static LED_MODE funnelMode = null;
  private static LED_MODE elevatorMode = null;

  CANdle ledcontroller = new CANdle(LEDConstants.candleCanid);
  // private int currentMode = 0;

  public LedSubsystem(Coral coral) { // If needed Add AlgaeRollers
    this.coral = coral;
    // this.algae = algae;
    SmartDashboard.putBoolean("coralOnBoardTest", true);
    SmartDashboard.putBoolean("isInRangeTest", true);
    SmartDashboard.putBoolean("ClimbedTest", false);

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
    // this.coralOnBoard = coralOnBoard;
  }

  @Override
  public void periodic() {

    // coralOnBoardTest = coral.holdingPiece(); //  Maybe need to add more
  //algaeOnBoard = algae.hasObject();  
  
    matchTime = DriverStation.getMatchTime();
    inAuton = DriverStation.isAutonomousEnabled();
    inTeleop = DriverStation.isTeleopEnabled();


    // if (inTeleop == false && endgameWarningStarted == false && matchTime > 0) {
    // setColor("DEFAULT", 0, 2, "SOLID");
    // }

    coralOnBoardTest = SmartDashboard.getBoolean("coralOnBoardTest", true);
    isInRangeTest = SmartDashboard.getBoolean("isInRangeTest", true);
    climbedTest = SmartDashboard.getBoolean("ClimbedTest", false);

    SmartDashboard.putBoolean("Bad Controller", badController());
    // SmartDashboard.putBoolean("In Auton", inAuton);
    // SmartDashboard.putBoolean("In Teleop", inTeleop);
    SmartDashboard.putNumber("Match Time", matchTime);
    // SmartDashboard.putBoolean("badController", badController());

    if (badController()) {
      if (elevatorMode != LED_MODE.BADCONTROLLER) {
        elevatorMode = LED_MODE.BADCONTROLLER;
        setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR, LED_PATTERN.STROBE);
      }
    } else if (inTeleop || inAuton) {
      if (!initialClearRan) {
        setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.CLEAR);
        setColor(LED_COLOR.OFF, LED_SECTION.ELEVATOR, LED_PATTERN.CLEAR);
        initialClearRan = true;
      }
      if (inTeleop) {
        // FIXME: Climber Logic

        if (matchTime <= LEDConstants.endgameWarning) {
          if (matchTime <= LEDConstants.endgameAlert) {
            if (elevatorMode != LED_MODE.END_GAME_ALERT) {
              elevatorMode = LED_MODE.END_GAME_ALERT;
              setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR, LED_PATTERN.STROBE);
            }
          } else if (elevatorMode != LED_MODE.END_GAME_WARNING) {
            elevatorMode = LED_MODE.END_GAME_WARNING;
            setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR, LED_PATTERN.SOLID);
          }
        }
        if (climbed == true) {
          if (funnelMode != LED_MODE.CLIMBED && matchTime > 0) {
            funnelMode = LED_MODE.CLIMBED;
            setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.RAINBOW);
          }
        } else if (coralOnBoardTest && isInRangeTest) {
          if (funnelMode != LED_MODE.CORAL_READY) {
            funnelMode = LED_MODE.CORAL_READY;
            setColor(LED_COLOR.GREEN, LED_SECTION.FUNNEL, LED_PATTERN.FLASH);
          }
        } else if (coralOnBoardTest) {
          if (funnelMode != LED_MODE.CORAL_ON_BOARD) {
            funnelMode = LED_MODE.CORAL_ON_BOARD;
            setColor(LED_COLOR.WHITE, LED_SECTION.FUNNEL, LED_PATTERN.SOLID);
          }

        }
        // Just In Case we need algaeOnBoard (Not Necessary)
        // else if (algaeOnBoard) {
        // if (funnelMode != LED_MODE.ALGAE_ON_BOARD){
        // funnelMode = LED_MODE.ALGAE_ON_BOARD;
        // setColor(LED_COLOR.GREEN,LED_SECTION.FUNNEL, LED_PATTERN.STROBE);
        // }
        // }
        else if (funnelMode != LED_MODE.DEFAULT) {
          funnelMode = LED_MODE.DEFAULT;
          setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.CLEAR);
        }

      }
    }
  }

  private void defaultColors() {
    ledcontroller.clearAnimation(0);
    ledcontroller.clearAnimation(1);
    ledcontroller.clearAnimation(2);
    ledcontroller.clearAnimation(3);
    setColor(LED_COLOR.PURPLE, LED_SECTION.ELEVATOR, LED_PATTERN.FLASH);
    ledcontroller.animate(
        new LarsonAnimation(255, 0, 255, 0, 0.75, LEDConstants.numLED, LarsonAnimation.BounceMode.Back, 14), 0);
    // ledcontroller.animate(
    // new LarsonAnimation(255, 0, 255, 0, 0.50, LEDConstants.numLED,
    // LarsonAnimation.BounceMode.Back, 7), 1);
  }

  private boolean badController() {
    if (!DriverStation.isJoystickConnected(0) || !DriverStation.isJoystickConnected(1)) {
      return true;
    }

    String joystick1Name = DriverStation.getJoystickName(1).toLowerCase();

    return !DriverStation.getJoystickName(0).contains("InterLinkDX") &&
        !( (DriverConstants.operatorController == JoystickChoice.PS5 &&
                joystick1Name.contains("dualsense")));
  }

  public void setColor(LED_COLOR color, LED_SECTION section, LED_PATTERN pattern) {

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
      case WHITE:
        r = 255;
        g = 255;
        b = 255;
        break;
      case OFF:
        r = 0;
        g = 0;
        b = 0;
        break;
    }

    switch (section) {
      case FUNNEL:
        offsetLED = LEDConstants.funnelOffset;
        nbrLED = LEDConstants.funnelNumLED;
        selectedSlot = 0;
        ledcontroller.clearAnimation(0);
        break;
      case ELEVATOR:
        offsetLED = LEDConstants.elevatorOffset;
        nbrLED = LEDConstants.elevatorNumLED;
        selectedSlot = 1;
        ledcontroller.clearAnimation(1);
        break;
    }

    switch (pattern) {
      case SOLID:
        ledcontroller.setLEDs(r, g, b, 0, offsetLED, nbrLED);
        break;
      case FLASH:

        ledcontroller.animate(new SingleFadeAnimation(r, g, b, 0, LEDConstants.flashSpeed, nbrLED, offsetLED),
            selectedSlot);
        break;
      case STROBE:
        ledcontroller.animate(new StrobeAnimation(r, g, b, 0, LEDConstants.strobeSpeed, nbrLED, offsetLED),
            selectedSlot);
        break;
      case TWINKLE:
        ledcontroller.animate(new TwinkleAnimation(r, g, b, 0, 0.5, nbrLED, TwinklePercent.Percent42, offsetLED),
            selectedSlot);
        break;
      case CLEAR:
        ledcontroller.setLEDs(0, 0, 0, 0, offsetLED, nbrLED);
        break;
      case LARSON:
        ledcontroller.animate(

            new LarsonAnimation(r, g, b, 0, 0.75, LEDConstants.numLED, LarsonAnimation.BounceMode.Back, 7),
            selectedSlot);
        break;
      case RAINBOW:
        ledcontroller.animate(new RainbowAnimation(1, 0.9, nbrLED, true, offsetLED), selectedSlot);
        break;

    }
  }
}
