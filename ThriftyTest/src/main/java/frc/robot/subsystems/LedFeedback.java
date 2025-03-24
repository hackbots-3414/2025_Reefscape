
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
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ButtonBindingConstants;
import frc.robot.Constants.ButtonBindingConstants.ButtonBoardChoice;
import frc.robot.Constants.CanRangeConstants;
import frc.robot.Constants.CommandBounds;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.RobotObserver;

public class LedFeedback extends SubsystemBase {
    @SuppressWarnings("unused")
    private final Logger m_logger = LoggerFactory.getLogger(LedFeedback.class);
    private static double matchTime = 0;
    // private Supplier<Boolean> isInRange;
    // private boolean algaeOnBoard = false;
    private boolean coralOnBoard = false;
    private boolean coralInRange = false;
    private boolean climbed = false;
    private boolean algaeOnBoard = false;
    private boolean algaeInRange = false;
    private boolean algaeTooClose = false;
    private boolean inAuton = false;
    private boolean inTeleop = false;
    private double rangeLeft = 0.0;
    private double rangeRight = 0.0;
    private CommandSwerveDrivetrain drivetrain;

    private int selectedSlot = 0;

    private static enum LED_MODE {
        CORAL_ON_BOARD, CORAL_READY, END_GAME_WARNING, END_GAME_ALERT, DEFAULT,
        BADCONTROLLER, IN_RANGE, CLIMBED, ALGAE_ON_BOARD, DEFAULT_ENDGAME, ALGAE_READY, ALGAE_TOO_CLOSE, ALIGNED, CLOSE,
        ALIGNED_REEF, ALIGNED_BRANCH, ALIGNED_RIGHT, ALIGNED_LEFT;
    };

    private static enum LED_COLOR {
        RED, YELLOW, GREEN, PURPLE, BLUE, WHITE, OFF, BROWN, ORANGE, BLUE_VIOLET, DEEP_PINK;
    };

    private static enum LED_PATTERN {
        TWINKLE, STROBE, LARSON, FLASH, SOLID, CLEAR, RAINBOW;
    };

    private static enum LED_SECTION {
        FUNNEL_LEFT, ELEVATOR_LEFT, FUNNEL_RIGHT, ELEVATOR_RIGHT;
    }

    private static LED_MODE mode = null;

    private CANdle ledcontroller = new CANdle(IDConstants.candle1);
    private CANdle ledcontroller2 = new CANdle(IDConstants.candle2);

    public LedFeedback(CommandSwerveDrivetrain m_drivetrain) {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.7; // dim the LEDs to 70% brightness
        ledcontroller.configAllSettings(config, 20);
        ledcontroller2.configAllSettings(config, 20);
        this.drivetrain = m_drivetrain;


        // SmartDashboard.putBoolean("AlignedRight", alignedRight());
        // SmartDashboard.putBoolean("AlignedLeft", alignedLeft());
        // SmartDashboard.putBoolean("coralOnBoard", coralOnBoard);
        // SmartDashboard.putBoolean("coralInRange", coralInRange);
        // SmartDashboard.putBoolean("algaeOnBoard", algaeOnBoard);
        // SmartDashboard.putBoolean("ALGAEINRANGE", algaeInRange);
        // SmartDashboard.putBoolean("ALGAETOOCLOSE", algaeTooClose);
        // SmartDashboard.putBoolean("CLIMBED", climbed);
        defaultColors();

    }

    @Override
    public void periodic() {
        matchTime = (DriverStation.getMatchType() == MatchType.None) ? Double.POSITIVE_INFINITY : DriverStation.getMatchTime();
        inAuton = DriverStation.isAutonomousEnabled();
        inTeleop = DriverStation.isTeleopEnabled();

        rangeRight =  drivetrain.getRangeRightDistance();
        rangeLeft =  drivetrain.getRangeLeftDistance();
        
        coralOnBoard = RobotObserver.getCoralPieceHeld();
        algaeOnBoard = RobotObserver.getAlgaePieceHeld();
        coralInRange = CommandBounds.reefBounds.isActive();
        algaeInRange = CommandBounds.netBounds.isActive();
        algaeTooClose = CommandBounds.netBounds.isActive(); // need to change
        climbed = RobotObserver.getClimbed();

        if (badController()) {

            if (mode != LED_MODE.BADCONTROLLER) {
                mode = LED_MODE.BADCONTROLLER;
                setAll(LED_COLOR.RED, LED_PATTERN.STROBE);

            }
        } 
        else if (inTeleop || inAuton) {
            if (climbed) {
                if (mode != LED_MODE.CLIMBED) {
                    if (matchTime < LedConstants.endgameWarning) {
                        setAll(LED_COLOR.OFF, LED_PATTERN.RAINBOW);
                    }
                    mode = LED_MODE.CLIMBED;

                }

                // Check if Coral on Board and At reef and not aligned to Branch
            }
            // Check for endgame
           else if (matchTime <= LedConstants.endgameWarning) {
                // Check for Final Seconds of  Endgame
                if (matchTime <= LedConstants.endgameAlert) {
                    if (mode != LED_MODE.END_GAME_ALERT) {
                        mode = LED_MODE.END_GAME_ALERT;
                        setAll(LED_COLOR.YELLOW, LED_PATTERN.STROBE);
                    }
                    // In Start of Endgame
                } else {
                    if (mode != LED_MODE.END_GAME_WARNING) {
                        mode = LED_MODE.END_GAME_WARNING;
                        setAll(LED_COLOR.YELLOW, LED_PATTERN.SOLID);

                    }
                }
            }
            else if (coralOnBoard && alignedReef()) {
                if (mode != LED_MODE.ALIGNED_REEF) {
                    mode = LED_MODE.ALIGNED_REEF;
                    setAll(LED_COLOR.ORANGE, LED_PATTERN.STROBE);

                }
                // Check if Aligned too far right or Left Branch on Reef
                // } else if (coralOnBoard && (alignedRight() || alignedLeft())) {
                // if (mode != LED_MODE.ALIGNED_BRANCH) {
                // mode = LED_MODE.ALIGNED_BRANCH;
                // setAll(LED_COLOR.GREEN, LED_PATTERN.FLASH);
                // }
            } else if (coralOnBoard && alignedRight()) {
                if (mode != LED_MODE.ALIGNED_RIGHT) {
                    mode = LED_MODE.ALIGNED_RIGHT;
                    setRight(LED_COLOR.DEEP_PINK, LED_PATTERN.FLASH);

                }
            } else if (coralOnBoard && alignedLeft()) {
                if (mode != LED_MODE.ALIGNED_LEFT) {
                    mode = LED_MODE.ALIGNED_LEFT;
                    setLeft(LED_COLOR.DEEP_PINK, LED_PATTERN.FLASH);

                }
            }
            // Check if Coral is On Board and In range of reef
            else if (coralOnBoard && coralInRange) {
                if (mode != LED_MODE.CORAL_READY) {
                    mode = LED_MODE.CORAL_READY;
                    setAll(LED_COLOR.BLUE, LED_PATTERN.SOLID);

                }
                // Check if Coral is On Board
            } else if (coralOnBoard) {
                if (mode != LED_MODE.CORAL_ON_BOARD) {
                    mode = LED_MODE.CORAL_ON_BOARD;
                    setAll(LED_COLOR.WHITE, LED_PATTERN.SOLID);

                }   
                // Check if Algae is On Board and Too Close to Net
            } else if (algaeOnBoard && algaeTooClose) {
                if (mode != LED_MODE.ALGAE_TOO_CLOSE) {
                    mode = LED_MODE.ALGAE_TOO_CLOSE;
                    setAll(LED_COLOR.BROWN, LED_PATTERN.STROBE);

                }
                // Check if Algae is On Board and In range to Net
            } else if (algaeOnBoard && algaeInRange) {
                if (mode != LED_MODE.ALGAE_READY) {
                    mode = LED_MODE.ALGAE_READY;
                    setAll(LED_COLOR.BLUE, LED_PATTERN.SOLID);
                }
                // Check if Algae is on board
            } else if (algaeOnBoard) {
                if (mode != LED_MODE.ALGAE_ON_BOARD) {
                    mode = LED_MODE.ALGAE_ON_BOARD;
                    setAll(LED_COLOR.GREEN, LED_PATTERN.SOLID);

                }
                // If Everything is false make it Default Colors
            } else {
                if (mode != LED_MODE.DEFAULT) {
                    defaultColors();
                    mode = LED_MODE.DEFAULT;
                }
            }

            // Run this when robot is disabled
        } else {
            if (mode != LED_MODE.DEFAULT) {
                defaultColors();
                mode = LED_MODE.DEFAULT;
                return;
            }
        }
    }

    private boolean alignedReef() {
        return (Math.abs(rangeRight
                - CanRangeConstants.closeAlignedDistanceMeters) < CanRangeConstants.closeAlignedDistanceMeters
                        * CanRangeConstants.tolerance
                &&
                Math.abs(rangeLeft
                        - CanRangeConstants.closeAlignedDistanceMeters) < CanRangeConstants.closeAlignedDistanceMeters
                                * CanRangeConstants.tolerance);
    }

    private boolean alignedLeft() {
        return (Math.abs(rangeRight
                - CanRangeConstants.closeAlignedDistanceMeters) < CanRangeConstants.closeAlignedDistanceMeters
                        * CanRangeConstants.tolerance)
                &&
                (Math.abs(rangeLeft
                        - CanRangeConstants.farAlignedDistanceMeters) < CanRangeConstants.farAlignedDistanceMeters
                                * CanRangeConstants.tolerance);
    }

    private boolean alignedRight() {
        return (Math.abs(
                rangeLeft - CanRangeConstants.closeAlignedDistanceMeters) < CanRangeConstants.closeAlignedDistanceMeters
                        * CanRangeConstants.tolerance)
                &&
                (Math.abs(rangeRight
                        - CanRangeConstants.farAlignedDistanceMeters) < CanRangeConstants.farAlignedDistanceMeters
                                * CanRangeConstants.tolerance);
    }

    private void clearAllAnimations() {
        ledcontroller.clearAnimation(0);
        ledcontroller.clearAnimation(1);
        ledcontroller2.clearAnimation(0);
        ledcontroller2.clearAnimation(1);
    }

    private void defaultColors() {
        clearAllAnimations();
        setElevator(LED_COLOR.PURPLE, LED_PATTERN.FLASH); // changed to heartbeat mode
        setFunnel(LED_COLOR.PURPLE, LED_PATTERN.LARSON);
    }

    private boolean badController() {
        boolean driverConnected = DriverStation.isJoystickConnected(ButtonBindingConstants.driverPort);
        boolean operatorConnected = DriverStation.isJoystickConnected(ButtonBindingConstants.buttonBoardPort);

        if (!driverConnected || !operatorConnected)
            return true;

        String driverName = DriverStation.getJoystickName(ButtonBindingConstants.driverPort).toLowerCase();
        String operatorName = DriverStation.getJoystickName(ButtonBindingConstants.buttonBoardPort).toLowerCase();

        boolean driverOk = driverName.contains(ButtonBindingConstants.dragonReinsName) || driverName.contains(ButtonBindingConstants.driverBackupName);

        boolean operatorOk = (ButtonBindingConstants.buttonBoardChoice == ButtonBoardChoice.BUTTONBOARD)
                ? operatorName.contains(ButtonBindingConstants.buttonBoardName)
                : operatorName.contains(ButtonBindingConstants.operatorBackupName);

        return !(driverOk && operatorOk);
    }

    public void setAll(LED_COLOR color, LED_PATTERN pattern) {
        clearAllAnimations();
        setColor(color, LED_SECTION.ELEVATOR_LEFT, pattern); // changed to heartbeat mode
        setColor(color, LED_SECTION.ELEVATOR_RIGHT, pattern); // changed to heartbeat mode
        setColor(color, LED_SECTION.FUNNEL_LEFT, pattern);
        setColor(color, LED_SECTION.FUNNEL_RIGHT, pattern);
    }

    public void setElevator(LED_COLOR color, LED_PATTERN pattern) {
        setColor(color, LED_SECTION.ELEVATOR_LEFT, pattern);
        setColor(color, LED_SECTION.ELEVATOR_RIGHT, pattern);
    }

    public void setFunnel(LED_COLOR color, LED_PATTERN pattern) {
        setColor(color, LED_SECTION.FUNNEL_LEFT, pattern);
        setColor(color, LED_SECTION.FUNNEL_RIGHT, pattern);
    }

    public void setRight(LED_COLOR color, LED_PATTERN pattern) {
        setColor(color, LED_SECTION.ELEVATOR_RIGHT, pattern);
        setColor(color, LED_SECTION.FUNNEL_RIGHT, pattern);
    }

    public void setLeft(LED_COLOR color, LED_PATTERN pattern) {
        setColor(color, LED_SECTION.ELEVATOR_LEFT, pattern);
        setColor(color, LED_SECTION.FUNNEL_LEFT, pattern);
    }

    @SuppressWarnings("incomplete-switch")
    public void setColor(LED_COLOR color, LED_SECTION section, LED_PATTERN pattern) {
        int nbrLED = 0;
        int offsetLED = 0;

        CANdle candle = null;
        Color c = new Color();

        switch (color) {
            case BLUE -> c = Color.kBlue;
            case GREEN -> c = Color.kGreen;
            case RED -> c = Color.kRed;
            case YELLOW -> c = Color.kYellow;
            case PURPLE -> c = Color.kPurple;
            case WHITE -> c = Color.kWhite;
            case BROWN -> c = Color.kBrown;
            case ORANGE -> c = Color.kOrange;
            case DEEP_PINK -> c = Color.kDeepPink;
            case OFF -> c = Color.kBlack;

        }

        int r = (int) (c.red * 255);
        int b = (int) (c.blue * 255);
        int g = (int) (c.green * 255);

        switch (section) {
            case FUNNEL_LEFT:
                offsetLED = LedConstants.funnelOffset;
                nbrLED = LedConstants.funnelNumLED;
                selectedSlot = 0;
                candle = ledcontroller;
                break;
            case ELEVATOR_LEFT:
                offsetLED = LedConstants.elevatorOffset;
                nbrLED = LedConstants.elevatorNumLED;
                selectedSlot = 1;
                candle = ledcontroller;
                break;
            case FUNNEL_RIGHT:
                offsetLED = LedConstants.funnelOffset2;
                nbrLED = LedConstants.funnelNumLED2;
                selectedSlot = 0;
                candle = ledcontroller2;
                break;
            case ELEVATOR_RIGHT:
                offsetLED = LedConstants.elevatorOffset2;
                nbrLED = LedConstants.elevatorNumLED2;
                selectedSlot = 1;
                candle = ledcontroller2;
        }

        candle.clearAnimation(selectedSlot);

        switch (pattern) {
            case SOLID -> candle.setLEDs(r, g, b, 0, offsetLED, nbrLED);
            case FLASH -> candle.animate(
                    new SingleFadeAnimation(r, g, b, 0, LedConstants.flashSpeed, nbrLED, offsetLED), selectedSlot);
            case STROBE -> candle.animate(new StrobeAnimation(r, g, b, 0, LedConstants.strobeSpeed, nbrLED, offsetLED),
                    selectedSlot);
            case TWINKLE -> candle.animate(
                    new TwinkleAnimation(r, g, b, 0, 0.5, nbrLED, TwinklePercent.Percent42, offsetLED), selectedSlot);
            case CLEAR -> candle.setLEDs(0, 0, 0, 0, offsetLED, nbrLED);
            case LARSON -> candle.animate(new LarsonAnimation(r, g, b, 0, 0.5, nbrLED, BounceMode.Back, 7, offsetLED),
                    selectedSlot);
            case RAINBOW -> candle.animate(new RainbowAnimation(1, 0.9, nbrLED, true, offsetLED), selectedSlot);
        }
    }
}
