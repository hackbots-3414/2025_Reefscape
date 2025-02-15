package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.RobotObserver;

public class PathPlannerOverride extends Command {
    private final Pose2d goal;

    public PathPlannerOverride(Pose2d pose) {
        this.goal = pose;
    }

    @Override
    public void initialize() {
        TeleopCommand.setGoalPose(goal);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("ENDED POSE CHANGE", goal.toString());
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = RobotObserver.getPose();
        double translationDiff = pose.getTranslation().getDistance(goal.getTranslation());
        double rotationDiff = pose.getRotation().getDegrees() - goal.getRotation().getDegrees();

        SmartDashboard.putNumber("** TRANSLATION DIFF", translationDiff);
        SmartDashboard.putNumber("** ROTATION DIFF", rotationDiff);

        return  (Math.abs(translationDiff) < AutonConstants.overrideTolerance)
            &&  (Math.abs(rotationDiff) < AutonConstants.degreeTolerance);
    }
}
