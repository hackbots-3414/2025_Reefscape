// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.algaeTracking.AlgaeTracker;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackAlgae extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final AlgaeTracker tracker;

  private int timer;

  /** Creates a new TrackAlgae. */
  public TrackAlgae(CommandSwerveDrivetrain drivetrain, AlgaeTracker tracker) {
    this.drivetrain = drivetrain;
    this.tracker = tracker;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  private final double speed = 0.06;
  private final double speedMax = 0.1;
  @Override
  public void execute() {
    tracker.track().ifPresentOrElse(state -> {
      drivetrain.applyRRVelocities(new Transform2d(new Translation2d(Math.min(speed / state.size() * 100, speedMax), 0).rotateBy(state.rot()), state.rot().times(2)));
      timer = 0;
    }, () -> {
      if (++timer > 20) {
        drivetrain.stop();
      }
    });;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
