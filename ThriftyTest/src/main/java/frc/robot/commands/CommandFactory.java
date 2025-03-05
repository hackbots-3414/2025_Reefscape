package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.CommandBounds;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.RobotContainer.CoralLocationPresets;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSetpoints;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotSetpoints;
import frc.robot.utils.Shape;

/*
 * This simply sequences commands together in a nicer way.
 * This doesn't require an actual instance of itself - it's more of a utility
 * class to store related command-building methods.
 */

public class CommandFactory {
    /**
     * Ejects an algae piece.
     * This command does not end - rather, you have to cancel it.
     */
    public static Command algaeEject(AlgaeRollers rollers) {
        return new AlgaeEjectCommand(rollers);
    }

    /**
     * Intakes an algae piece from the specified height
     * This command begins by setting the pivot and elevator to the correct
     * position.
     * It then waits for both to finish, then activates the intake motors.
     * If we are picking up from the reef, the elevator and pivot will not stow
     * until the robot has left the reef's general area - as to avoid an ugly
     * collision with the reef.
     * Finally, the pivot and elevator are stowed, and the algae rollers will
     * hold a piece, if present, otherwise stop.
     * @param elevator
     */
    public static Command algaeIntake(
        Elevator elevator,
        AlgaeRollers rollers,
        Pivot pivot,
        AlgaeLocationPresets position
    ) {
        ElevatorSetpoints elevatorPosition = ElevatorSetpoints.fromAlgaePreset(position);
        PivotSetpoints pivotPosition = PivotSetpoints.fromAlgaePreset(position);
        // build a command sequence
        SequentialCommandGroup sequence = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, elevatorPosition),
                new PivotCommand(pivot, pivotPosition)
            ),
            new AlgaeIntakeCommand(rollers)
        );
        // we don't want to stow right away, because we will collide with the
        // reef. However, we want to wait until we leave the reef area before
        // doing any stowing.
        if (position == AlgaeLocationPresets.ALGAE_L2
            || position == AlgaeLocationPresets.ALGAE_L3) {
            sequence.addCommands(
                new WaitUntilCommand(CommandBounds.reefBounds::isInactive)
            );
        }
        // final stow
        sequence.finallyDo(() -> {
            elevator.stow();
            pivot.stow();
        });
        return sequence;
    }

    public static Command stow(
        Elevator elevator,
        Pivot pivot
    ) {
        return new ParallelCommandGroup(
            new PivotCommand(pivot, PivotSetpoints.STOW),
            new ElevatorCommand(elevator, ElevatorSetpoints.STOW)
        );
    }

    /**
     * Scores an algae in the nearest location
     * If outside of the bounds for that location, nothing happens.
     * The command ends when the algae is scored and the elevators are reset
     */
    public static Command algaeScore(
        Elevator elevator,
        Pivot pivot,
        AlgaeRollers rollers,
        AlgaeLocationPresets position
    ) {
        ElevatorSetpoints elevatorPosition = ElevatorSetpoints.fromAlgaePreset(position);
        PivotSetpoints pivotPosition = PivotSetpoints.fromAlgaePreset(position);
        // get the boundaries
        Shape boundary;
        switch (position) {
            case ALGAE_L2, ALGAE_L3 -> boundary = CommandBounds.reefBounds;
            case NET -> boundary = CommandBounds.netBounds;
            case PROCESSOR -> boundary = CommandBounds.processorBounds;
            default -> boundary = null;
        }
        BooleanSupplier limiter = () -> { return false; };
        if (boundary != null) {
            limiter = boundary::isActive;
        }
        // actually build a command
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, elevatorPosition),
                new PivotCommand(pivot, pivotPosition)
            ),
            new AlgaeEjectCommand(rollers).withTimeout(Constants.AlgaeRollerConstants.algaeEjectTime)
        )
            .onlyIf(limiter)
            .finallyDo(elevator::stow)
            .finallyDo(pivot::stow);
    }

    /**
     * Creates a coral intake command.
     * This command sets the elevator to stow, then runs the coral intake.
     * This command finished when there is a coral on board.
     */
    public static Command coralIntake(
        CoralRollers coral,
        Elevator elevator
    ) {
        return new SequentialCommandGroup(
            new ElevatorCommand(elevator, ElevatorSetpoints.STOW),
            new CoralIntakeCommand(coral)
        );
    }

    /**
     * Creates a coral ejecting command
     * This command runs the coral motors in reverse, to eject a coral that
     * may have gotten stuck in the robot. First, the elevator is set to stow,
     * and will not continue until the elevator is there.
     * This command ends after the coral is completely out of the intake system,
     * i.e. not seen by either IR sensor. Do not expect this command to fully
     * remove coral from the robot.
     */
    public static Command coralEject(
            CoralRollers coral,
            Elevator elevator
    ) {
        return new SequentialCommandGroup(
            new ElevatorCommand(elevator, ElevatorSetpoints.STOW),
            new CoralUnjamCommand(coral)
        );
    }

    /**
     * Creates a coral scoring command
     * This command sets the elevator to the correct position, then runs coral
     * eject. Finally, the elevator is stowed.
     * This command ends after the coral is released.
     * This command will only run if you are within bounds for the reef (as it
     * is the only scoring location)
     */
    public static Command coralScore(
        Elevator elevator,
        CoralRollers coral,
        CoralLocationPresets scorePosition
    ) {
        return new SequentialCommandGroup(
            new ElevatorCommand(elevator, ElevatorSetpoints.fromCoralPreset(scorePosition)),
            new CoralUnjamCommand(coral)
        )
            .onlyIf(CommandBounds.reefBounds::isActive)
            .finallyDo(elevator::stow);
    }
}
