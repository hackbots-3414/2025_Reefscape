package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.CommandBounds;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.commands.ElevatorCommand.ElevatorPosition;
import frc.robot.commands.PivotCommand.PivotPosition;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
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
        return new AlgaeRollerEjectCommand(rollers);
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
        ElevatorPosition elevatorPosition = ElevatorPosition
            .fromAlgaePreset(position);
        PivotPosition pivotPosition = PivotPosition
            .fromAlgaePreset(position);
        // build a command sequence
        SequentialCommandGroup sequence = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, elevatorPosition),
                new PivotCommand(pivot, pivotPosition)
            ),
            new AlgaeRollerIntakeCommand(rollers)
        );
        // we don't want to stow right away, because we will collide with the
        // reef. However, we want to wait until we leave the reef area before
        // doing any stowing.
        if (position == AlgaeLocationPresets.REEFUPPER
            || position == AlgaeLocationPresets.REEFLOWER) {
            sequence.addCommands(
                new WaitUntilCommand(CommandBounds.reefBounds::isActive)
            );
        }
        // final stow
        sequence.addCommands(
            stow(elevator, pivot)
        );
        return sequence;
    }

    public static Command stow(
        Elevator elevator,
        Pivot pivot
    ) {
        return new ParallelCommandGroup(
            new PivotCommand(pivot, PivotPosition.Stow),
            new ElevatorCommand(elevator, ElevatorPosition.Stow)
        );
    }

    public static Command algaeScore(
        Elevator elevator,
        Pivot pivot,
        AlgaeRollers rollers,
        AlgaeLocationPresets position
    ) {
        ElevatorPosition elevatorPosition = ElevatorPosition
            .fromAlgaePreset(position);
        PivotPosition pivotPosition = PivotPosition
            .fromAlgaePreset(position);
        // get the boundaries
        Shape boundary;
        switch (position) {
            case REEFLOWER,REEFUPPER -> boundary = CommandBounds.reefBounds;
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
            new AlgaeRollerEjectCommand(rollers)
                .withTimeout(Constants.AlgaeRollerConstants.algaeEjectTime),
            stow(elevator, pivot)
        ).onlyIf(limiter);
    }
}
