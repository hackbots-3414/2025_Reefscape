package frc.robot;

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
    private Pose2d m_pose;

    public static void setPose(Pose2d pose) {
        getInstance().m_pose = pose;
    }

    public static Pose2d getPose() {
        return getInstance().m_pose;
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
    private boolean m_visionExpired;

    public static void setVisionExpired(boolean expired) {
        getInstance().m_visionExpired = expired;
    }

    public static boolean getVisionExpired() {
        return getInstance().m_visionExpired;
    }
}
