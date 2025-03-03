
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ButtonBindingConstants;
import frc.robot.Constants.CommandBounds;
import frc.robot.Constants.IDConstants;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.ButtonBindingConstants.ButtonBoardChoice;
import frc.robot.Constants.ButtonBindingConstants.DriverChoice;
import frc.robot.RobotObserver;

public class LedFeedback extends SubsystemBase {
    private static double matchTime = 0;
    // private Supplier<Boolean> isInRange;
    // private boolean coralOnBoard = false;
    // private boolean algaeOnBoard = false;
    // private boolean climbed = false;
    private boolean coralOnBoard = false;
    private boolean inRange = false;
    private boolean climbed = false;
    private int r = 0;
    private int g = 0;
    private int b = 0;
    private int offsetLED = 0;
    private int nbrLED = 0;

    private boolean inAuton = false;
    private boolean inTeleop = false;

    private int selectedSlot = 0;
    private boolean initialClearRan = false;

    private static enum LED_MODE {
        CORAL_ON_BOARD, CORAL_READY, END_GAME_WARNING, END_GAME_ALERT, DEFAULT,
        BADCONTROLLER, IN_RANGE, CLIMBED, ALGAE_ON_BOARD, DEFAULT_ENDGAME;
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

    CANdle ledcontroller = new CANdle(IDConstants.candle1);
    CANdle ledcontroller2 = new CANdle(IDConstants.candle2);

    public LedFeedback() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.7; // dim the LEDs to 70% brightness
        ledcontroller.configAllSettings(config);
        ledcontroller2.configAllSettings(config);
        defaultColors(ledcontroller);
        defaultColors(ledcontroller2);

    }

    @Override
    public void periodic() {
        matchTime = DriverStation.getMatchTime();
        inAuton = DriverStation.isAutonomousEnabled();
        inTeleop = DriverStation.isTeleopEnabled();

        coralOnBoard = RobotObserver.getPieceHeld();
        inRange = CommandBounds.reefBounds.isActive();
        climbed = RobotObserver.getClimbed();

        if (badController()) {
            if (elevatorMode != LED_MODE.BADCONTROLLER) {
                elevatorMode = LED_MODE.BADCONTROLLER;
                setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR, LED_PATTERN.STROBE, ledcontroller);
                setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR, LED_PATTERN.STROBE, ledcontroller2);

            }
        } else if (inTeleop || inAuton) {
            if (!initialClearRan) {
                setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.CLEAR, ledcontroller);
                setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.CLEAR, ledcontroller2);
                initialClearRan = true;
            }
            if (inTeleop) {
                if (matchTime <= LedConstants.endgameWarning) {
                    if (matchTime <= LedConstants.endgameAlert) {
                        if (elevatorMode != LED_MODE.END_GAME_ALERT) {
                            elevatorMode = LED_MODE.END_GAME_ALERT;
                            setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR, LED_PATTERN.STROBE, ledcontroller);
                            setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR, LED_PATTERN.STROBE, ledcontroller2);

                        }
                    } else if (elevatorMode != LED_MODE.END_GAME_WARNING) {
                        elevatorMode = LED_MODE.END_GAME_WARNING;
                        setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR, LED_PATTERN.SOLID, ledcontroller);
                        setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR, LED_PATTERN.SOLID, ledcontroller2);

                    }
                }
                if (climbed) {
                    if (funnelMode != LED_MODE.CLIMBED && matchTime > 0) {
                        funnelMode = LED_MODE.CLIMBED;
                        setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.RAINBOW, ledcontroller);
                        setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.RAINBOW, ledcontroller2);

                    }
                } else if (coralOnBoard && inRange) {
                    if (funnelMode != LED_MODE.CORAL_READY) {
                        funnelMode = LED_MODE.CORAL_READY;
                        setColor(LED_COLOR.GREEN, LED_SECTION.FUNNEL, LED_PATTERN.FLASH, ledcontroller);
                        setColor(LED_COLOR.GREEN, LED_SECTION.FUNNEL, LED_PATTERN.FLASH, ledcontroller2);

                    }
                } else if (coralOnBoard) {
                    if (funnelMode != LED_MODE.CORAL_ON_BOARD) {
                        funnelMode = LED_MODE.CORAL_ON_BOARD;
                        setColor(LED_COLOR.WHITE, LED_SECTION.FUNNEL, LED_PATTERN.SOLID, ledcontroller);
                        setColor(LED_COLOR.WHITE, LED_SECTION.FUNNEL, LED_PATTERN.SOLID, ledcontroller2);

                    }

                } else if (funnelMode != LED_MODE.DEFAULT_ENDGAME && matchTime < LedConstants.endgameWarning) {
                    funnelMode = LED_MODE.DEFAULT_ENDGAME;
                    setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.SOLID, ledcontroller2);
                    setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.SOLID, ledcontroller);

                } else if (elevatorMode != LED_MODE.DEFAULT) {
                    elevatorMode = LED_MODE.DEFAULT;
                    setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.SOLID, ledcontroller2);
                    setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.SOLID, ledcontroller);
                    setColor(LED_COLOR.PURPLE, LED_SECTION.ELEVATOR, LED_PATTERN.FLASH, ledcontroller);
                    setColor(LED_COLOR.PURPLE, LED_SECTION.ELEVATOR, LED_PATTERN.FLASH, ledcontroller2);
                }

            }
        }
    }

    private void defaultColors(CANdle ledcontroller) {
        ledcontroller.clearAnimation(0);
        ledcontroller.clearAnimation(1);
        setColor(LED_COLOR.PURPLE, LED_SECTION.ELEVATOR, LED_PATTERN.FLASH, ledcontroller);

        ledcontroller.animate(
                new LarsonAnimation(255, 0, 255, 0, 0.75, LedConstants.elevatorOffset, LarsonAnimation.BounceMode.Back,
                        14),
                0);
    }

    private boolean badController() {
        boolean driverConnected = DriverStation.isJoystickConnected(ButtonBindingConstants.driverPort);
        boolean operatorConnected = DriverStation.isJoystickConnected(ButtonBindingConstants.buttonBoardPort);

        if (!driverConnected || !operatorConnected) return true;

        String driverName = DriverStation.getJoystickName(ButtonBindingConstants.driverPort).toLowerCase();
        String operatorName = DriverStation.getJoystickName(ButtonBindingConstants.buttonBoardPort).toLowerCase();

        boolean driverOk = (ButtonBindingConstants.driverChoice == DriverChoice.DRAGONREINS) ?
            driverName.contains(ButtonBindingConstants.dragonReinsName)
            :
            driverName.contains(ButtonBindingConstants.driverBackupName);
        
        boolean operatorOk = (ButtonBindingConstants.buttonBoardChoice == ButtonBoardChoice.BUTTONBOARD) ?
            operatorName.contains(ButtonBindingConstants.buttonBoardName)
            :
            operatorName.contains(ButtonBindingConstants.operatorBackupName);

        return !driverOk || !operatorOk;
    }

    public void setColor(LED_COLOR color, LED_SECTION section, LED_PATTERN pattern, CANdle ledcontroller) {

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
                offsetLED = LedConstants.funnelOffset;
                nbrLED = LedConstants.funnelNumLED;
                selectedSlot = 0;
                ledcontroller.clearAnimation(0);
                break;
            case ELEVATOR:
                offsetLED = LedConstants.elevatorOffset;
                nbrLED = LedConstants.elevatorNumLED;
                selectedSlot = 1;
                ledcontroller.clearAnimation(1);
                break;
        }

        switch (pattern) {
            case SOLID:
                ledcontroller.setLEDs(r, g, b, 0, offsetLED, nbrLED);
                break;
            case FLASH:

                ledcontroller.animate(new SingleFadeAnimation(r, g, b, 0, LedConstants.flashSpeed, nbrLED, offsetLED),
                        selectedSlot);
                break;
            case STROBE:
                ledcontroller.animate(new StrobeAnimation(r, g, b, 0, LedConstants.strobeSpeed, nbrLED, offsetLED),
                        selectedSlot);
                break;
            case TWINKLE:
                ledcontroller.animate(
                        new TwinkleAnimation(r, g, b, 0, 0.5, nbrLED, TwinklePercent.Percent42, offsetLED),
                        selectedSlot);
                break;
            case CLEAR:
                ledcontroller.setLEDs(0, 0, 0, 0, offsetLED, nbrLED);
                break;
            case LARSON:
                ledcontroller.animate(
                        new LarsonAnimation(r, g, b, 0, 0.75, LedConstants.numLED, LarsonAnimation.BounceMode.Back, 7),
                        selectedSlot);
                break;
            case RAINBOW:
                ledcontroller.animate(new RainbowAnimation(1, 0.9, nbrLED, true, offsetLED), selectedSlot);
                break;

        }
    }
}
