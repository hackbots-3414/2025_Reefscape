package frc.robot.driveassist;

import java.security.KeyStore.Entry;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotObserver;

public class Autopilot {
    private static final Logger m_logger = LoggerFactory.getLogger(Autopilot.class);
    private Constraints m_constraints;

    private final double dt = 0.020;

    public Autopilot(Constraints constraints) {
        m_constraints = constraints;
    }

    public Translation2d adjust(
        Pose2d current,
        Pose2d target,
        Translation2d velocity
    ) {
        // Rotation2d entryAngle = target.getRotation();
        Rotation2d entryAngle = Rotation2d.kZero;
        // direction and distance to actual target
        Translation2d offset = target.getTranslation().minus(current.getTranslation());
        double distanceToTarget = offset.getNorm();
        if (distanceToTarget == 0) return Translation2d.kZero;
        // create new target
        Translation2d entryDirection = new Translation2d(
                entryAngle.getCos(),
                entryAngle.getSin()
        );
        offset = entryDirection.times(-distanceToTarget / 2);
        Pose2d newTarget = new Pose2d(offset, Rotation2d.kZero).plus(new Transform2d(target.getTranslation(), Rotation2d.kZero));
        offset = newTarget.getTranslation().minus(current.getTranslation());
        RobotObserver.getField().getObject("foo").setPose(newTarget);
        double distance = offset.getNorm();
        Translation2d directionI = offset.div(distance);
        Translation2d directionU = new Translation2d(
            directionI.getY(),
            -directionI.getX()
        );
        // current velocity (i & j)
        double veloI = project(velocity, directionI);
        double veloU = project(velocity, directionU);
        // ideal state
        double idealI = Math.sqrt(2 * m_constraints.m_decceleration * distance);
        double idealU = 0.0;
        // drive towards goal state
        double adjustedI = approach(idealI, veloI);
        double adjustedU = approach(idealU, veloU);
        SmartDashboard.putNumber("ri", adjustedI);
        SmartDashboard.putNumber("ru", adjustedU);

        Translation2d adjusted = directionI.times(adjustedI).plus(
            directionU.times(adjustedU));
        
        return adjusted;
    }

    private double project(Translation2d vector, Translation2d axis) {
        double dot = vector.getX() * axis.getX() + vector.getY() * axis.getY();
        return dot / Math.pow(axis.getNorm(), 2);
    }

    private double approach(double goal, double initial) {
        if (Math.abs(goal - initial) < dt * m_constraints.m_acceleration) {
            // we're within range, just adjust to what we need.
            return goal;
        }
        if (goal > initial) {
            return initial + dt * m_constraints.m_acceleration;
        } else {
            return initial - dt * m_constraints.m_acceleration;
        }
    }

    /* Constraints to limit autopilot */
    public static class Constraints {
        private double m_acceleration;
        private double m_decceleration;

        public Constraints() {}

        public Constraints(double acceleration, double decceleration) {
            m_acceleration = acceleration;
            m_decceleration = decceleration;
        }

        public Constraints withAcceleration(double acceleration) {
            m_acceleration = acceleration;
            return this;
        }

        public Constraints withDecceleration(double decceleration) {
            m_decceleration = decceleration;
            return this;
        }
    }
}
