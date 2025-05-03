package frc.robot.binding;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonBindingConstants;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.ButtonBindingConstants.Operator;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.states.Climb;
import frc.robot.superstructure.states.ClimbRaised;
import frc.robot.superstructure.states.CompleteCoralIntake;
import frc.robot.superstructure.states.CoralEject;
import frc.robot.superstructure.states.CoralIntake;
import frc.robot.superstructure.states.CoralScore;
import frc.robot.superstructure.states.CoralScoreReady;
import frc.robot.superstructure.states.ElevatorZero;
import frc.robot.superstructure.states.FunnelOpened;
import frc.robot.superstructure.states.GroundAlgaeIntake;
import frc.robot.superstructure.states.HighGroundAlgaeIntake;
import frc.robot.superstructure.states.UpperReefAlgaeIntake;
import frc.robot.superstructure.states.ReefAlign.ReefSide;
import frc.robot.superstructure.states.LowerReefAlgaeIntake;
import frc.robot.superstructure.states.Net;
import frc.robot.superstructure.states.NetReady;
import frc.robot.superstructure.states.Processor;
import frc.robot.superstructure.states.ProcessorReady;
import frc.robot.superstructure.states.ReefAlign;
import frc.robot.superstructure.states.Stowed;

public class OperatorBindings implements Binder {
  private final CommandPS5Controller m_controller =
      new CommandPS5Controller(ButtonBindingConstants.operatorPort);

  private final Trigger m_l1 = m_controller.button(Operator.L1);
  private final Trigger m_secondaryL1 = m_controller.button(Operator.secondaryL1);
  private final Trigger m_l2 = m_controller.button(Operator.L2);
  private final Trigger m_l3 = m_controller.button(Operator.L3);
  private final Trigger m_l4 = m_controller.button(Operator.L4);

  private final Trigger m_left = m_controller.button(Operator.leftReef);
  private final Trigger m_right = m_controller.button(Operator.rightReef);

  private final Trigger m_coralIntake = m_controller.button(Operator.intake);
  private final Trigger m_ejectCoral = m_controller.button(Operator.ejectCoral);

  private final Trigger m_algae = m_controller.button(Operator.algaeModeButton);

  private final Trigger m_algaeGround = m_controller.button(Operator.ground);
  private final Trigger m_algaeHighGround = m_controller.button(Operator.highGround);
  private final Trigger m_algaeLowReef = m_controller.button(Operator.lowAlgae);
  private final Trigger m_algaeHighReef = m_controller.button(Operator.highAlgae);

  private final Trigger m_processor = m_controller.button(Operator.processor);
  private final Trigger m_net = m_controller.button(Operator.net);

  private final Trigger m_climberUp = m_controller.button(Operator.climbUp);
  private final Trigger m_climb = m_controller.button(Operator.climb);

  private final Trigger m_stow = m_controller.button(Operator.stow);

  private final Trigger m_funnelLeft = m_controller.button(Operator.leftFunnel);
  private final Trigger m_funnelRight = m_controller.button(Operator.rightFunnel);
  private final Trigger m_funnel = m_funnelLeft.and(m_funnelRight);

  private final Trigger m_zeroElevator = m_controller.button(Operator.zeroElevator);

  public void bind(Superstructure superstructure) {
    /* algae intake */
    m_algae.and(m_algaeGround).whileTrue(superstructure.enter(new GroundAlgaeIntake()));
    m_algae.and(m_algaeHighGround).whileTrue(superstructure.enter(new HighGroundAlgaeIntake()));
    m_algae.and(m_algaeLowReef).whileTrue(superstructure.enter(new LowerReefAlgaeIntake()));
    m_algae.and(m_algaeHighReef).whileTrue(superstructure.enter(new UpperReefAlgaeIntake()));

    /* algae score */
    m_algae.and(m_processor).whileTrue(superstructure.enter(new ProcessorReady()));
    m_algae.and(m_processor).onFalse(superstructure.enter(new Processor()));
    m_algae.and(m_net).whileTrue(superstructure.enter(new NetReady()));
    m_algae.and(m_net).onFalse(superstructure.enter(new Net()));

    /* coral intake & score */
    m_coralIntake.whileTrue(superstructure.enter(new CoralIntake()));
    m_coralIntake.onFalse(superstructure.enter(new CompleteCoralIntake()));
    m_ejectCoral.whileTrue(superstructure.enter(new CoralEject()));
    bindCoral(m_l1.and(m_secondaryL1.negate()), CoralLevel.L1, superstructure);
    bindCoral(m_l1.and(m_secondaryL1), CoralLevel.SecondaryL1, superstructure);
    bindCoral(m_l2, CoralLevel.L2, superstructure);
    bindCoral(m_l3, CoralLevel.L3, superstructure);
    bindCoral(m_l4, CoralLevel.L4, superstructure);

    /* align */
    m_left.whileTrue(superstructure.enter(new ReefAlign(ReefSide.Left)));
    m_right.whileTrue(superstructure.enter(new ReefAlign(ReefSide.Right)));

    /* climb */
    m_climb.whileTrue(superstructure.enter(new Climb()));
    m_climberUp.whileTrue(superstructure.enter(new ClimbRaised()));

    /* misc */
    m_zeroElevator.whileTrue(superstructure.enter(new ElevatorZero()));
    m_stow.whileTrue(superstructure.enter(new Stowed()));
    m_funnel.whileTrue(superstructure.enter(new FunnelOpened()));
  }

  private void bindCoral(Trigger trigger, CoralLevel level, Superstructure superstructure) {
    trigger.whileTrue(superstructure.enter(new CoralScoreReady(level)));
    trigger.and(superstructure.aligned()).onTrue(superstructure.enter(new CoralScore(level)));
    trigger.onFalse(superstructure.enter(new CoralScore(level)));
  }
}

