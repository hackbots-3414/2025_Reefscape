package frc.robot.superstructure.states;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ScoringLocationsLeft;
import frc.robot.Constants.ScoringLocationsRight;
import frc.robot.driveassist.Autopilot;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;
import frc.robot.utils.FieldUtils;

public class ReefAlign implements EnterableState {
  private List<Pose2d> m_locations;
  /**
   * Represents a state where the robot aligns to a reef face
   */
  public ReefAlign(ReefSide side) {
    if (side == ReefSide.Left) {
      m_locations = Arrays.stream(ScoringLocationsLeft.values())
        .map(location -> location.value)
        .collect(Collectors.toList());
    } else {
      m_locations = Arrays.stream(ScoringLocationsRight.values())
        .map(location -> location.value)
        .collect(Collectors.toList());
    }
  }

  public Command build(Subsystems subsystems) {
    return Commands.defer(() -> {
      List<Pose2d> locations = new ArrayList<>();
      m_locations.forEach(location -> locations.add(FieldUtils.getGlobalPose(location)));
      Autopilot.Target target = new Autopilot.Target()
        .withReference(subsystems.drivetrain().getPose().nearest(locations));
      return subsystems.drivetrain().align(DriveConstants.kTightAutopilot, target);
    }, Set.of(
      subsystems.drivetrain()));
  }

  public enum ReefSide { Left, Right; }
}
