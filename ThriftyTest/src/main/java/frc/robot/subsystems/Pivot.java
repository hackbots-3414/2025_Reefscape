package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.utils.StateSpaceController;

public class Pivot extends SubsystemBase {

  private final TalonFX pivot = new TalonFX(PivotConstants.pivotMotorID);
  private final CANcoder cancoder = new CANcoder(PivotConstants.EncoderID);

    private StateSpaceController<N2, N1, N2> pivotController;

  private double position;

  public Pivot() {
    configEncoder();
    configMotor();
    Vector<N2> initialState = getOutput();
    pivotController = new StateSpaceController<N2, N1, N2>(ElevatorConstants.k_config, this::getOutput, this::applyInput, initialState);
  }

  public void configEncoder() {
    cancoder.clearStickyFaults();
    cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(PivotConstants.absoluteSensorRange)
            .withSensorDirection(PivotConstants.cancoderInvert)
            .withMagnetOffset(PivotConstants.encoderOffset));

    cancoder.getConfigurator().apply(canCoderConfiguration, 0.2);
  }

  public void configMotor() {
    pivot.getConfigurator().apply(new TalonFXConfiguration(), 0.050);

    TalonFXConfiguration configuration = new TalonFXConfiguration()

        .withFeedback(new FeedbackConfigs()
            .withFeedbackRemoteSensorID(PivotConstants.EncoderID)
            .withFeedbackRotorOffset(PivotConstants.encoderOffset)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withRotorToSensorRatio(PivotConstants.rotorToSensorRatio)
            .withSensorToMechanismRatio(PivotConstants.sensorToMechanismRatio))
            
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(PivotConstants.motorInvert))
            
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(PivotConstants.forwardSoftLimitThreshold)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(PivotConstants.reverseSoftLimitThreshold)
            .withReverseSoftLimitEnable(true));

    pivot.getConfigurator().apply(configuration, 0.2);
  }

  private Vector<N2> getOutput() {
    double position = cancoder.getAbsolutePosition().getValueAsDouble(); // position is in mechanism rotations
    double velocity = cancoder.getVelocity().getValueAsDouble();
    return VecBuilder.fill(position, velocity);
  }

  private void applyInput(Vector<N1> inputs) {
    VoltageOut config = new VoltageOut(0);
    double volts = inputs.get(0);

    pivot.setControl(config.withOutput(volts));
  }

  public void setPosition(double goal) {pivotController.setReference(VecBuilder.fill(goal, 0.0));}
  public void stop() {setPosition(position);}
  public double getPosition() {return position;}

  public void setGroundPickup() {setPosition(PivotConstants.groundPickup);}
  public void setProcessor() {setPosition(PivotConstants.processor);}
  public void setReefPickup() {setPosition(PivotConstants.reefPickup);}
  public void setNet() {setPosition(PivotConstants.net);}

  @Override
  public void periodic() {
    position = cancoder.getAbsolutePosition().getValueAsDouble();
  }
}