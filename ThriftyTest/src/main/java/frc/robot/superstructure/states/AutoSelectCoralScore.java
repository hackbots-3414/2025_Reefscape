package frc.robot.superstructure.states;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CoralLevel;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.superstructure.EnterableState;
import frc.robot.superstructure.Superstructure.Subsystems;

public class AutoSelectCoralScore implements EnterableState {


  public Command build(Subsystems subsystems) {
    final Map<ElevatorState, Command> scoreCommands = Map.ofEntries(
        Map.entry(ElevatorState.L1,
            new CoralScore(CoralLevel.L1).build(subsystems)
                .onlyIf(subsystems.elevator().ready(ElevatorState.L1))),
        Map.entry(ElevatorState.L2,
            new CoralScore(CoralLevel.L2).build(subsystems)
                .onlyIf(subsystems.elevator().ready(ElevatorState.L2))),
        Map.entry(ElevatorState.L3,
            new CoralScore(CoralLevel.L3).build(subsystems)
                .onlyIf(subsystems.elevator().ready(ElevatorState.L3))),
        Map.entry(ElevatorState.L4, new CoralScore(CoralLevel.L4).build(subsystems)
            .onlyIf(subsystems.elevator().ready(ElevatorState.L4))));
    return Commands.select(scoreCommands, subsystems.elevator()::getElevatorState);
  }
}
