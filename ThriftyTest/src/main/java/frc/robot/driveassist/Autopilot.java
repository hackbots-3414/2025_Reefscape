package frc.robot.driveassist;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autopilot {
    private static final Logger m_logger = LoggerFactory.getLogger(Autopilot.class);
    private Constraints m_constraintsU;
    private Constraints m_constraintsI;

    private final double dt = 0.020;

    public Autopilot(Constraints constraintsI, Constraints constraintsU) {
        m_constraintsI = constraintsI;
        m_constraintsU = constraintsU;
        SmartDashboard.putNumber("div", 1);
    }

    public Translation2d adjust(
        Pose2d current,
        Pose2d target,
        Translation2d velocity
    ) {
        Rotation2d entryAngle = target.getRotation();
        // direction and distance to actual target
        Translation2d offset = target.getTranslation().minus(current.getTranslation());
        double distance = offset.getNorm();
        if (distance == 0) return Translation2d.kZero;
        // create new target
        Translation2d entryDirection = new Translation2d(
                entryAngle.getCos(),
                entryAngle.getSin()
        );
        Translation2d directionI = offset.div(distance);
        Translation2d directionU = new Translation2d(
            directionI.getY(),
            -directionI.getX()
        );
        // current velocity (i & j)
        double veloI = project(velocity, directionI);
        double veloU = project(velocity, directionU);
        // double veloU = 0.0;
        Translation2d entry = entryDirection.times(-distance);
        double entryDistance = project(entry, directionU);
        // drive towards goal state
        double adjustedI = approach(distance, veloI, m_constraintsI);
        double adjustedU = approach(entryDistance, veloU, m_constraintsU);
        // combine
        Translation2d adjusted = directionI.times(adjustedI).plus(
            directionU.times(adjustedU));
        return adjusted;
    }

    private double project(Translation2d vector, Translation2d axis) {
        double dot = vector.getX() * axis.getX() + vector.getY() * axis.getY();
        return dot / Math.pow(axis.getNorm(), 2);
    }

    private double approach(double distance, double initial, Constraints c) {
        double goal = Math.sqrt(2 * c.m_decceleration * Math.abs(distance)) * Math.signum(distance);
        if (Math.abs(goal - initial) < dt * c.m_acceleration) {
            // we're within range, just adjust to what we need.
            return goal;
        }
        // check for a "out-of-bounds" position
        if (goal < initial && goal > 0) return goal;
        if (goal > initial && goal < 0) return goal;
        if (goal > initial) {
            return initial + dt * c.m_acceleration;
        } else {
            return initial - dt * c.m_acceleration;
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
