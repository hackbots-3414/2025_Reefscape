package frc.robot;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Shape;

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
    private Field2d m_field;

    public static void setField(Field2d field) {
        getInstance().m_field = field;
        // A new field means that we should probably update smart dashboard
        SmartDashboard.putData("Super Field", getInstance().m_field);
    }

    public static Field2d getField() {
        return getInstance().m_field;
    }

    /* Keeps track of the latest time an april tag was seen */
    private Supplier<Boolean> m_visionExpiredSupplier;

    public static void setVisionExpiredSupplier(Supplier<Boolean> expiredSupplier) {
        getInstance().m_visionExpiredSupplier = expiredSupplier;
    }

    public static boolean getVisionExpired() {
        return getInstance().m_visionExpiredSupplier.get();
    }

    private boolean m_disableBounds;

    public static void setDisableBounds(boolean boundsDisabled) {
        getInstance().m_disableBounds = boundsDisabled;
    }

    public static boolean getDisableBounds() {
        return getInstance().m_disableBounds;
    }

    /* Check for drive in pose range & vision works */
    private Function<Shape, Boolean> m_shapeChecker;

    public static void setShapeChecker(Function<Shape, Boolean> shapeChecker) {
        getInstance().m_shapeChecker = shapeChecker;
    }

    public static Function<Shape, Boolean> getShapeChecker() {
        return getInstance().m_shapeChecker;
    }

    /* Check for drive in pose range & vision works */
    private Function<Pose2d, Pose2d> m_poseFlipper;

    public static void setPoseFlipper(Function<Pose2d, Pose2d> poseFlipper) {
        getInstance().m_poseFlipper = poseFlipper;
    }

    public static Function<Pose2d, Pose2d> getPoseFlipper() {
        return getInstance().m_poseFlipper;
    }
}
