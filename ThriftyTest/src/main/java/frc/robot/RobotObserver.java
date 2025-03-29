package frc.robot;

import java.util.Optional;
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

    /* Pose2d to watch the pose of the robot and associated methods */
    private Supplier<Pose2d> m_poseSupplier;

    public static void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        getInstance().m_poseSupplier = poseSupplier;
    }

    public static Pose2d getPose() {
        return getInstance().m_poseSupplier.get();
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

    /* Keeps track of the latest time an april tag was seen */
    private Supplier<Boolean> m_visionValidSupplier;

    public static void setVisionValidSupplier(Supplier<Boolean> visionValidSupplier) {
        getInstance().m_visionValidSupplier = visionValidSupplier;
    }

    public static boolean getVisionValid() {
        return getInstance().m_visionValidSupplier.get();
    }

    private Supplier<Double> m_elevatorHeightSupplier;

    public static void setElevatorHeightSupplier(Supplier<Double> visionValidSupplier) {
        getInstance().m_elevatorHeightSupplier = visionValidSupplier;
    }

    public static double getElevatorHeightSupplier() {
        // return getInstance().m_elevatorHeightSupplier.get();
        return 0.0;
    }

    private boolean m_reefMode = false;

    public static void setReefMode(boolean enabled) {
        getInstance().m_reefMode = enabled;
    }

    public static boolean getReefMode() {
        return getInstance().m_reefMode;
    }

    private ReefClipLocations m_reefClipLocation = ReefClipLocations.LEFT;

    public static void setReefClipLocation(ReefClipLocations reefClipLocation) {
        getInstance().m_reefClipLocation = reefClipLocation;
    }

    public static ReefClipLocations getReefClipLocation() {
        return getInstance().m_reefClipLocation;
    }

    private BooleanSupplier m_coralPieceHeldSupplier;
    private BooleanSupplier m_algaeHeldSupplier;

    public static void setPieceHeldSupplier(BooleanSupplier pieceHeldSupplier) {
        getInstance().m_coralPieceHeldSupplier = pieceHeldSupplier;
    }
    
    public static void setAlgaePieceHeldSupplier(BooleanSupplier algaeHeldSupplier){
        getInstance().m_algaeHeldSupplier = algaeHeldSupplier;
    }

    public static boolean getCoralPieceHeld() {
        // return getInstance().m_coralPieceHeldSupplier.getAsBoolean();
        return false;
    }

    public static boolean getAlgaePieceHeld() {
        // return getInstance().m_algaeHeldSupplier.getAsBoolean();
        return false;
    }

    private boolean m_climbed = false;

    public static void setClimbed(boolean climbed) {
        getInstance().m_climbed = climbed;
    }

    public static boolean getClimbed() {
        // return getInstance().m_climbed;
        return false;
    }

    private DoubleSupplier m_rangeDistanceSupplier;

    public static void setRangeDistanceSupplier(DoubleSupplier rangeDistanceSupplier) {
        getInstance().m_rangeDistanceSupplier = rangeDistanceSupplier;
    }
    
    public static double getRangeDistance(){
        return getInstance().m_rangeDistanceSupplier.getAsDouble();
    }

    private Supplier<Optional<Double>> m_compDistanceSupplier;

    public static void setCompensationDistanceSupplier(Supplier<Optional<Double>> sup) {
        getInstance().m_compDistanceSupplier = sup;
    }

    public static Optional<Double> getCompensationDistance() {
        return getInstance().m_compDistanceSupplier.get();
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
        // return getInstance().m_noElevatorSup.getAsBoolean();
        return false;
    }

    private BooleanSupplier m_reefReadySupplier;

    public static void setReefReadySupplier(BooleanSupplier rangeSupplier) {
        getInstance().m_reefReadySupplier = rangeSupplier;
    }

    public static boolean getReefReady() {
        return false;
        // return getInstance().m_reefReadySupplier.getAsBoolean() && getCoralPieceHeld();
    }
}
