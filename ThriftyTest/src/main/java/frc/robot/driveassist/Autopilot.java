package frc.robot.driveassist;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.pathplanner.lib.util.FlippingUtil;

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
        Translation2d velocity,
        Target target
    ) {
        Pose2d reference = target.m_reference;
        Rotation2d entryAngle = target.m_entryAngle;
        // direction and distance to actual target
        Translation2d offset = reference.getTranslation().minus(current.getTranslation());
        double distance = offset.getNorm();
        if (distance == 0) return Translation2d.kZero;
        // create new target
        Translation2d entryDirection = new Translation2d(
                entryAngle.getCos(),
                entryAngle.getSin()
        );
        // end velocity
        Translation2d endVelocity = entryDirection.times(target.m_velocity);
        // calculate directions for i & j
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
        double adjustedI = approach(distance, veloI, m_constraintsI) + Math.max(project(endVelocity, directionI), 0);
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

    /* End States (the goal to reach) */
    public static class Target {
        private Pose2d m_reference;
        private Rotation2d m_entryAngle;
        private double m_velocity;

        /**
         * Creates a blank autopilot target with reference (0,0) and rotation
         * of zero.
         * <b>Note:</b> an entry angle <i>MUST</i> be specified, and this doesn't
         * set one. Your code will crash without one.
         */
        public Target() {
            m_reference = Pose2d.kZero;
            m_velocity = 0;
        }

        /**
         * Constructs an autopilot target with a given pose and entry angle
         */
        public Target(Pose2d reference, Rotation2d entryAngle) {
            m_reference = reference;
            m_entryAngle = entryAngle;
            m_velocity = 0;
        }

        /**
         * Constructs an autopilot target with a given pose.
         * Entry angle is set to be the rotation of the pose.
         */
        public Target(Pose2d reference) {
            m_reference = reference;
            m_entryAngle = reference.getRotation();
            m_velocity = 0;
        }

        /**
         * Modifies this instance's reference pose and returns itself for
         * easier method chaining. <i>NOTE:</i> This also sets, if unset, the
         * entry angle to be the angle of the pose.
         */
        public Target withReference(Pose2d reference) {
            m_reference = reference;
            if (m_entryAngle == null) m_entryAngle = reference.getRotation();
            return this;
        }

        /**
         * Modifies this instance's entry angle and returns itself for easier
         * method chaining
         */
        public Target withEntryAngle(Rotation2d entryAngle) {
            m_entryAngle = entryAngle;
            return this;
        }

        /**
         * Modifies this instance's end velocity and returns itself for easier
         * method chaining
         */
        public Target withVelocity(double velocity) {
            m_velocity = velocity;
            return this;
        }

        /**
         * Returns this target's reference pose
         */
        public Pose2d getReference() {
            return m_reference;
        }

        /**
         * Returns this target's desired entry angle
         */
        public Rotation2d getEntryAngle() {
            return m_entryAngle;
        }

        /**
         * Returns this target/s end velocity
         */
        public double getVelocity() {
            return m_velocity;
        }

        /**
         * Flips a target across the field, preserving relative entry angle
         * and rotation.
         */
        public Target flip() {
            m_reference = FlippingUtil.flipFieldPose(m_reference);
            m_entryAngle = FlippingUtil.flipFieldRotation(m_entryAngle);
            return this;
        }
    }
}
