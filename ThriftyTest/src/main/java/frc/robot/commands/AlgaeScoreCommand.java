package frc.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.RobotContainer.AlgaeLocationPresets;
import frc.robot.RobotObserver;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class AlgaeScoreCommand extends Command {
  private final Logger m_logger = LoggerFactory.getLogger(AlgaeScoreCommand.class);

  private final AlgaeRollers rollers;
  private final Elevator elevator;
  private final Pivot pivot;
  private final AlgaeLocationPresets location;
  private boolean isDone;
  private double initialTime;

  public AlgaeScoreCommand(AlgaeRollers rollers, Elevator elevator, Pivot pivot, AlgaeLocationPresets location) {
    this.rollers = rollers;
    this.elevator = elevator;
    this.pivot = pivot;
    this.location = location;
    addRequirements(rollers, elevator, pivot);
  }

  @Override
  public void initialize() {
    isDone = false;
    initialTime = Utils.getCurrentTimeSeconds();
    switch (location) {
      case NET -> {
        m_logger.trace("Setting NET");
        elevator.setNet();
        pivot.setNet();
      }
      case PROCESSOR -> {
        elevator.setProcessor();
        pivot.setProcessor();
      }
      default -> isDone = true;
    }
  }

  @Override
  public void execute() {
    m_logger.trace("Running execute");
    if (elevator.atSetpoint() || RobotObserver.getNoElevatorZone() && pivot.atSetpoint()) {
      m_logger.trace("Everything's at setpoint");
      if (location == AlgaeLocationPresets.NET) {
        m_logger.trace("Ejecting");
        rollers.ejectAlgae();
      } else {
        m_logger.trace("Scoring location is {}", location);
      }
    } else {
      initialTime = Utils.getCurrentTimeSeconds();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_logger.trace("ENDING");
    if (location == AlgaeLocationPresets.NET) {
      m_logger.trace("Stowing and releasing");
      pivot.setStow();
    }
    elevator.release();
    rollers.smartStop();
  }

  @Override
  public boolean isFinished() {
    m_logger.trace("isdone = {}", isDone);
    return isDone || (Utils.getCurrentTimeSeconds() - initialTime) >= AlgaeRollerConstants.algaeEjectTime;
  }
}
