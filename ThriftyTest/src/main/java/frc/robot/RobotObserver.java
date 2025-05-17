package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ReefClipLocations;

public class RobotObserver {
    private static RobotObserver m_instance;

    private static RobotObserver getInstance() {
        if (m_instance == null) {
            m_instance = new RobotObserver();
        }
        return m_instance;
    }

    /* the velocity of the robot */
    private DoubleSupplier m_veloSupplier;

    public static void setVelocitySupplier(DoubleSupplier veloSupplier) {
        getInstance().m_veloSupplier = veloSupplier;
    }

    public static double getVelocity() {
        return getInstance().m_veloSupplier.getAsDouble();
    }

    /* Field2d to display important details about the robot */
    private Field2d m_field = new Field2d();

    public static void setField(Field2d field) {
        getInstance().m_field = field;
        // A new field means that we should probably update smart dashboard
        SmartDashboard.putData("Super Field", getInstance().m_field);
    }

    public static Field2d getField() {
        return getInstance().m_field;
    }

    private BooleanSupplier m_coralPieceHeldSupplier;
    private BooleanSupplier m_algaeHeldSupplier;

    public static void setCoralHeldSupplier(BooleanSupplier pieceHeldSupplier) {
        getInstance().m_coralPieceHeldSupplier = pieceHeldSupplier;
    }
    
    public static void setAlgaePieceHeldSupplier(BooleanSupplier algaeHeldSupplier){
        getInstance().m_algaeHeldSupplier = algaeHeldSupplier;
    }

    public static boolean getCoralPieceHeld() {
        return getInstance().m_coralPieceHeldSupplier.getAsBoolean();
    }

    public static boolean getAlgaePieceHeld() {
        return getInstance().m_algaeHeldSupplier.getAsBoolean();
    }

    private boolean m_climbed = false;

    public static void setClimbed(boolean climbed) {
        getInstance().m_climbed = climbed;
    }

    public static boolean getClimbed() {
        return getInstance().m_climbed;
    }

    private Supplier<Pose2d> m_antitargetSupplier;
    
    public static void setAntitargetSupplier(Supplier<Pose2d> antitargetSup) {
        getInstance().m_antitargetSupplier = antitargetSup;
    }

    public static Pose2d getAntitarget() {
        return getInstance().m_antitargetSupplier.get();
    }

    private BooleanSupplier m_ffEnabledSupplier; 

    public static void setFFEnabledSupplier(BooleanSupplier ffSupplier) {
        getInstance().m_ffEnabledSupplier = ffSupplier;
    }
    public static boolean getFFEnabled() {
        return getInstance().m_ffEnabledSupplier.getAsBoolean();
    }

    private BooleanSupplier m_noElevatorSup;

    public static void setNoElevatorZoneSupplier(BooleanSupplier noElevatorSup) {
        getInstance().m_noElevatorSup = noElevatorSup;
    }

    public static boolean getNoElevatorZone() {
        return getInstance().m_noElevatorSup.getAsBoolean();
    }

    private BooleanSupplier m_reefReadySupplier;

    public static void setReefReadySupplier(BooleanSupplier rangeSupplier) {
        getInstance().m_reefReadySupplier = rangeSupplier;
    }

    public static boolean getReefReady() {
        return getInstance().m_reefReadySupplier.getAsBoolean() && getCoralPieceHeld();
    }

    private BooleanSupplier m_alignedSupplier;

    public static void setAlginedSupplier(BooleanSupplier alignedSupplier) {
        getInstance().m_alignedSupplier = alignedSupplier;
    }

    public static boolean getAligned() {
        return getInstance().m_alignedSupplier.getAsBoolean();
    }

}
