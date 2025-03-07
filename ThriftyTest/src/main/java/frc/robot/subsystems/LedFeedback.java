
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ButtonBindingConstants;
import frc.robot.Constants.ButtonBindingConstants.ButtonBoardChoice;
import frc.robot.Constants.ButtonBindingConstants.DriverChoice;
import frc.robot.Constants.CommandBounds;
import frc.robot.Constants.IDConstants;
// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;
import frc.robot.Constants.LedConstants;
import frc.robot.RobotObserver;

public class LedFeedback extends SubsystemBase {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(LedFeedback.class);
    private static double matchTime = 0;
    // private Supplier<Boolean> isInRange;
    // private boolean algaeOnBoard = false;
    private boolean coralOnBoard = false;
    private boolean inRange = false;
    private boolean climbed = false;
    private int r = 0;
    private int g = 0;
    private int b = 0;

    private boolean inAuton = false;
    private boolean inTeleop = false;

    private int selectedSlot = 0;

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
        FUNNEL, ELEVATOR, FUNNEL2, ELEVATOR2;
    }

    private static LED_MODE mode = null;

    private CANdle ledcontroller = new CANdle(IDConstants.candle1);
    private CANdle ledcontroller2 = new CANdle(IDConstants.candle2);

    public LedFeedback() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.7; // dim the LEDs to 70% brightness
        ledcontroller.configAllSettings(config);
        ledcontroller2.configAllSettings(config);
        defaultColors();
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
            if (mode != LED_MODE.BADCONTROLLER) {
                mode = LED_MODE.BADCONTROLLER;
                setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR, LED_PATTERN.STROBE);
                setColor(LED_COLOR.RED, LED_SECTION.ELEVATOR2, LED_PATTERN.STROBE);
                setColor(LED_COLOR.RED, LED_SECTION.FUNNEL, LED_PATTERN.STROBE);
                setColor(LED_COLOR.RED, LED_SECTION.FUNNEL2, LED_PATTERN.STROBE);
            }
        } else if (inTeleop || inAuton) {
            if (inTeleop) {
                if (matchTime <= LedConstants.endgameWarning) {
                    if (matchTime <= LedConstants.endgameAlert) {
                        if (mode != LED_MODE.END_GAME_ALERT) {
                            mode = LED_MODE.END_GAME_ALERT;
                            setColor(LED_COLOR.YELLOW, LED_SECTION.ELEVATOR, LED_PATTERN.STROBE);
                            setColor(LED_COLOR.YELLOW, LED_SECTION.ELEVATOR2, LED_PATTERN.STROBE);
                        }
                    } else {
                        if (mode != LED_MODE.END_GAME_WARNING) {
                            mode = LED_MODE.END_GAME_WARNING;
                            setColor(LED_COLOR.YELLOW, LED_SECTION.ELEVATOR, LED_PATTERN.STROBE);
                            setColor(LED_COLOR.YELLOW, LED_SECTION.ELEVATOR2, LED_PATTERN.STROBE);
                        }
                    }
                }

                if (climbed) {
                    if (mode != LED_MODE.CLIMBED && matchTime > 0) {
                        mode = LED_MODE.CLIMBED;
                        setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL, LED_PATTERN.RAINBOW);
                        setColor(LED_COLOR.OFF, LED_SECTION.FUNNEL2, LED_PATTERN.RAINBOW);
                        setColor(LED_COLOR.OFF, LED_SECTION.ELEVATOR, LED_PATTERN.RAINBOW);
                        setColor(LED_COLOR.OFF, LED_SECTION.ELEVATOR2, LED_PATTERN.RAINBOW);

                    }
                } else if (coralOnBoard && inRange) {
                    if (mode != LED_MODE.CORAL_READY) {
                        mode = LED_MODE.CORAL_READY;
                        setColor(LED_COLOR.BLUE, LED_SECTION.FUNNEL, LED_PATTERN.STROBE);
                        setColor(LED_COLOR.BLUE, LED_SECTION.FUNNEL2, LED_PATTERN.STROBE);

                    }
                } else if (coralOnBoard) {
                    if (mode != LED_MODE.CORAL_ON_BOARD) {
                        mode = LED_MODE.CORAL_ON_BOARD;
                        setColor(LED_COLOR.GREEN, LED_SECTION.FUNNEL, LED_PATTERN.SOLID);
                        setColor(LED_COLOR.GREEN, LED_SECTION.FUNNEL2, LED_PATTERN.SOLID);
                    }
                } else {
                    if (mode != LED_MODE.DEFAULT) {
                        defaultColors();
                        mode = LED_MODE.DEFAULT;
                    }
                }

            }
        } else {
            if (mode != LED_MODE.DEFAULT) {
                defaultColors();
                mode = LED_MODE.DEFAULT;
            }
            
        }
    }

    private void defaultColors() {
        setColor(LED_COLOR.PURPLE, LED_SECTION.ELEVATOR, LED_PATTERN.FLASH); // changed to heartbeat mode
        setColor(LED_COLOR.PURPLE, LED_SECTION.ELEVATOR2, LED_PATTERN.FLASH); //  changed to heartbeat mode
        setColor(LED_COLOR.PURPLE, LED_SECTION.FUNNEL, LED_PATTERN.LARSON);
        setColor(LED_COLOR.PURPLE, LED_SECTION.FUNNEL2, LED_PATTERN.LARSON);      
    }

    private boolean badController() {
        boolean driverConnected = DriverStation.isJoystickConnected(ButtonBindingConstants.driverPort);
        boolean operatorConnected = DriverStation.isJoystickConnected(ButtonBindingConstants.buttonBoardPort);

        if (!driverConnected || !operatorConnected) return true;

        String driverName = DriverStation.getJoystickName(ButtonBindingConstants.driverPort).toLowerCase();
        String operatorName = DriverStation.getJoystickName(ButtonBindingConstants.buttonBoardPort).toLowerCase();

        boolean driverOk = (ButtonBindingConstants.driverChoice == DriverChoice.DRAGONREINS)
                ? driverName.contains(ButtonBindingConstants.dragonReinsName)
                : driverName.contains(ButtonBindingConstants.driverBackupName);

        boolean operatorOk = (ButtonBindingConstants.buttonBoardChoice == ButtonBoardChoice.BUTTONBOARD)
                ? operatorName.contains(ButtonBindingConstants.buttonBoardName)
                : operatorName.contains(ButtonBindingConstants.operatorBackupName);

        return !(driverOk && operatorOk);
    }

    public void setColor(LED_COLOR color, LED_SECTION section, LED_PATTERN pattern) {
        int nbrLED = 0;
        int offsetLED = 0;

        CANdle candle = null;

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
                candle = ledcontroller;
                break;
            case ELEVATOR:
                offsetLED = LedConstants.elevatorOffset;
                nbrLED = LedConstants.elevatorNumLED;
                selectedSlot = 1;
                candle = ledcontroller;
                break;
            case FUNNEL2:
                offsetLED = LedConstants.funnelOffset2;
                nbrLED = LedConstants.funnelNumLED2;
                selectedSlot = 0;
                candle = ledcontroller2;
                break;
            case ELEVATOR2:
                offsetLED = LedConstants.elevatorOffset2;
                nbrLED = LedConstants.elevatorNumLED2;
                selectedSlot = 1;
                candle = ledcontroller2;
        }

        switch (pattern) {
            case SOLID -> candle.setLEDs(r, g, b, 0, offsetLED, nbrLED);
            case FLASH -> candle.animate(new SingleFadeAnimation(r, g, b, 0, LedConstants.flashSpeed, nbrLED, offsetLED), selectedSlot);
            case STROBE -> candle.animate(new StrobeAnimation(r, g, b, 0, LedConstants.strobeSpeed, nbrLED, offsetLED), selectedSlot);
            case TWINKLE -> candle.animate(new TwinkleAnimation(r, g, b, 0, 0.5, nbrLED, TwinklePercent.Percent42, offsetLED), selectedSlot);
            case CLEAR -> candle.setLEDs(0, 0, 0, 0, offsetLED, nbrLED);
            case LARSON -> candle.animate(new LarsonAnimation(r, g, b, 0, 0.5, nbrLED, BounceMode.Back, 7, offsetLED), selectedSlot);
            case RAINBOW -> candle.animate(new RainbowAnimation(1, 0.9, nbrLED, true, offsetLED), selectedSlot);
        }
    }
}
