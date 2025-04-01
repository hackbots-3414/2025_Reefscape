package frc.robot.driveassist;


import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FFConstants;
import frc.robot.RobotObserver;

public class DriverAssist {
    private final Logger m_logger = LoggerFactory.getLogger(DriverAssist.class);

    /**
     * Returns the calculated drive command to apply to the drivetrian
     * @param velocity the robot's requested velocity, in field relative
     * units.
     * @param current The robot's current position on the field
     * @param antitarget The closest force field antitarget
     * @return the field relative adjusted velocity
     */
    public Translation2d  calculate(
        Translation2d velocity,
        Pose2d current,
        Pose2d antitarget
    ) {
        Translation2d x = antitarget.getTranslation().minus(current.getTranslation());
        double xNorm = x.getNorm();
        if (xNorm == 0) {
            m_logger.warn("position is exactly ff center");
            return velocity;
        }
        boolean inside = false;
        if (xNorm < FFConstants.k_radius) {
            inside = true;
        } else {
            x = x.times((xNorm - FFConstants.k_radius) / xNorm);
        }
        Translation2d t = x.times(
            Math.sqrt(2 * FFConstants.k_decceleration/ x.getNorm())
        );
        boolean active = t.getNorm() < DriveConstants.k_maxTeleopLinearSpeed
            && RobotObserver.getFFEnabled();
        if (!active) {
            RobotObserver.getField().getObject("FF").setPoses();
            return velocity;
        }
        RobotObserver.getField().getObject("FF").setPose(antitarget);
        return adjust(velocity, t, inside);
    }

    private Translation2d adjust(
        Translation2d c,
        Translation2d t,
        boolean inside
    ) {
        double d = dot(c, t);
        if (d <= 0) {
            return c;
        }
        double magI = d / t.getNorm();
        if (magI <= 0) {
            return c;
        }
        Translation2d i = t.times(magI / t.getNorm());
        Translation2d u = c.minus(i);
        Translation2d a = t.times(magI / DriveConstants.k_maxTeleopLinearSpeed);
        if (t.getNorm() <= 0 || inside) {
            return u;
        }
        return u.plus(a);
    }

    private double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }
}
