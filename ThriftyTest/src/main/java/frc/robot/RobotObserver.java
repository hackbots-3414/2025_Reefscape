package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    /* Lets us keep track of if safety is enabled */
    private boolean m_manualModeEnabled = false;

    public static void toggleManualMode() {
        getInstance().m_manualModeEnabled = !getInstance().m_manualModeEnabled;
        SmartDashboard.putBoolean("SAFETY MODE", getInstance().m_manualModeEnabled);
    }

    public static boolean getManualMode() {
        return getInstance().m_manualModeEnabled;
    }

    private Supplier<Double> m_elevatorHeightSupplier;

    public static void setElevatorHeightSupplier(Supplier<Double> visionValidSupplier) {
        getInstance().m_elevatorHeightSupplier = visionValidSupplier;
    }

    public static double getElevatorHeightSupplier() {
        return getInstance().m_elevatorHeightSupplier.get();
    }

    private Runnable m_setSingleTag = () -> {};
    private Runnable m_setMultiTag = () -> {};

    public static void setSingleTag() {
        getInstance().m_setSingleTag.run();
    }
    public static void setMultiTag() {
        getInstance().m_setMultiTag.run();
    }

    public static void setSingleTagRunnable(Runnable singleTag) {
        getInstance().m_setSingleTag = singleTag;
    }
    public static void setMultiTagRunnable(Runnable multiTag) {
        getInstance().m_setMultiTag = multiTag;
    }
}
