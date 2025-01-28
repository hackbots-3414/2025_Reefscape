package frc.robot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;

public class Constants {
    public static final class ClimberConstants {
        public static final MotorOutputConfigs currentConfigs = new MotorOutputConfigs();
        public static final int leftClimberMotorID = 1;
        public static final int rightClimberMotorID = 2;
        public static final double climberUpVolts = 1.0; //FIXME figure out actual values for the climber voltage.
        public static final double climberDownVolts = -1.0; //FIXME figure out actual values for the climber voltage.
    }
}
