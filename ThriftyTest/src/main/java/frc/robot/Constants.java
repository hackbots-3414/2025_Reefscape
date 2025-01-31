package frc.robot;

public class Constants {
    public static final double globalCanTimeout = 20;
    public static final class ClimberConstants {
        public static final int leftClimberMotorID = 1;
        public static final int rightClimberMotorID = 2;
        public static final double climberUpVolts = 1.0; //FIXME figure out actual values for the climber voltage.
        public static final double climberCurrentLimit = 80.0;
    }
}
