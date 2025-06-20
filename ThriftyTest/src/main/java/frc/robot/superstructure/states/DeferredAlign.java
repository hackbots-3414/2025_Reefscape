package frc.robot.superstructure.states;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ScoringLocationsCenter;
import frc.robot.Constants.ScoringLocationsLeft;
import frc.robot.Constants.ScoringLocationsRight;
import com.therekrab.autopilot.APTarget;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;
import frc.robot.utils.FieldUtils;

public class DeferredAlign implements EnterableState {
  private List<Pose2d> m_locations;

  /**
   * Represents a state where the robot aligns to a nonstatic location and then waits there
   */
  public DeferredAlign(AlignLocation side) {
    switch (side) {
      case Left -> {
        m_locations = Arrays.stream(ScoringLocationsLeft.values())
            .map(location -> location.value)
            .collect(Collectors.toList());
      }
      case Right -> {
        m_locations = Arrays.stream(ScoringLocationsRight.values())
            .map(location -> location.value)
            .collect(Collectors.toList());
      }
      case Center -> {
        m_locations = Arrays.stream(ScoringLocationsCenter.values())
            .map(location -> location.value)
            .collect(Collectors.toList());
      }
      case Intake -> {
        m_locations = List.of(FieldConstants.kLeftIntake, FieldConstants.kRightIntake);
      }
    }
  }

  public Command build(Subsystems subsystems) {
    return Commands.defer(() -> {
      List<Pose2d> locations = new ArrayList<>();
      m_locations.forEach(location -> locations.add(FieldUtils.getLocalPose(location)));
      Pose2d closest = subsystems.drivetrain().getPose().nearest(locations);
      APTarget target = new APTarget(closest)
          .withEntryAngle(closest.getRotation());
      return Commands.sequence(
          subsystems.drivetrain().align(DriveConstants.kTightAutopilot, target),
          Commands.idle(subsystems.drivetrain()));
    }, Set.of(subsystems.drivetrain()));
  }

  public enum AlignLocation {
    Left, Right, Center, Intake;
  }
}
