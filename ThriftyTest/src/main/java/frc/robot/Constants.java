package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.elevator.ElevatorState;

public class Constants {
  public static class IDConstants {
    public static final int candle1 = 5;
    public static final int candle2 = 6;
  }

  public static class SimConstants {
    public static final double k_simPeriodic = 0.005;
  }

  public static class RobotConstants {
    public static final Time globalCanTimeout = Milliseconds.of(20); // 20 milliseconds
  }

  public static class FieldConstants {
    public static final Distance kFieldWidth = Meters.of(8.05);
    public static final Distance kFieldLength = Meters.of(17.55);
    public static final Translation2d reefCenter = new Translation2d(4.5, 4.0);
    public static final double kReefReadyAuton = 2.6;
    public static final double kReefReady = 2.1;
    public static final Pose2d k_processor = new Pose2d(5.974, 1.16, Rotation2d.kCW_90deg);

    public static final Pose2d kRightIntake = new Pose2d(1.247, 0.950, Rotation2d.fromDegrees(55));
    public static final Pose2d kLeftIntake = new Pose2d(1.211, 7.016, Rotation2d.fromDegrees(-55));

    public static final Pose2d kGH = new Pose2d(5.791, 4.046, Rotation2d.k180deg);
    public static final Pose2d kIJ = new Pose2d(5.155, 5.194, Rotation2d.fromDegrees(-120));

    public static final Pose2d kBarge = new Pose2d(7.459, 4.717, Rotation2d.fromDegrees(21.0));

    public static final Pose2d kStartCenter = new Pose2d(7.076, 3.991, Rotation2d.kPi);
    public static final Pose2d kStartRight = new Pose2d(7.076, 1.883, Rotation2d.kPi);
    public static final Pose2d kStartLeft = new Pose2d(7.076, 6.027, Rotation2d.kPi);
  }

  public static final class StateSpaceConstants {
    public static final double k_dt = 0.01; // fast state space, please!
    public static final double k_maxVoltage = 12.0;
  }

  public static final class TalonFXConstants {
    public static final double nominalVoltageVolts = 12.0; // DC Volts
    public static final double stallTorqueNewtonMeters = 4.69; // Nm
    public static final double stallCurrentAmps = 257.0; // Amps
    public static final double freeCurrentAmps = 1.5; // Amps
    public static final double freeSpeedRadPerSec = 6380.0 * 2.0 * Math.PI / 60.0; // RPM * 2pi / 60
                                                                                   // = Rad per
                                                                                   // second

    public static final double positionStdDevs = 1.0 / 2048.0; // rotations
    public static final double velocityStdDevs = 2.0 / 2048.0; // rotations

    public static final DCMotor TalonFXDCMotor =
        new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters,
            stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, 1);
  }

  public static final class KrakenX60Constants {
    public static final double nominalVoltageVolts = 12.0;
    public static final double stallTorqueNewtonMeters = 7.16;
    public static final double stallCurrentAmps = 374.38;
    public static final double freeCurrentAmps = 2.0;
    public static final double freeSpeedRadPerSec = Units.rotationsToRadians(6000);
    public static final double positionStdDevs = 1.0 / 2048.0;
    public static final double velocityStdDevs = 2.0 / 2048.0;

    public static final DCMotor KrakenX60Motor =
        new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters,
            stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, 1);
  }

  public static final class KrakenX60FOCConstants {
    public static final double nominalVoltageVolts = 12.0;
    public static final double stallTorqueNewtonMeters = 9.37;
    public static final double stallCurrentAmps = 483;
    public static final double freeCurrentAmps = 2.0;
    public static final double freeSpeedRadPerSec = Units.rotationsToRadians(5800);
    public static final double positionStdDevs = 1.0 / 2048.0;
    public static final double velocityStdDevs = 2.0 / 2048.0;

    public static final DCMotor KrakenX60FOCMotor =
        new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters,
            stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, 1);
  }

  public enum ScoringLocations {
    A(new Pose2d(3.188, 4.191, Rotation2d.fromDegrees(0))), // GOOD
    B(new Pose2d(3.188, 3.861, Rotation2d.fromDegrees(0))), // GOOD

    C(new Pose2d(3.72, 2.982, Rotation2d.fromDegrees(58.7))), // GOOD
    D(new Pose2d(3.967, 2.810, Rotation2d.fromDegrees(58.2))), // GOOD

    E(new Pose2d(4.998, 2.816, Rotation2d.fromDegrees(120))), // GOOD
    F(new Pose2d(5.283, 2.981, Rotation2d.fromDegrees(120))), // GOOD

    G(new Pose2d(5.791, 3.861, Rotation2d.fromDegrees(180))), // GOOD
    H(new Pose2d(5.791, 4.191, Rotation2d.fromDegrees(180))), // GOOD

    I(new Pose2d(5.283, 5.071, Rotation2d.fromDegrees(-120))), // GOOD
    J(new Pose2d(4.998, 5.236, Rotation2d.fromDegrees(-120))), // GOOD

    K(new Pose2d(3.951, 5.236, Rotation2d.fromDegrees(-60))), // GOOD
    L(new Pose2d(3.696, 5.071, Rotation2d.fromDegrees(-60))), // GOOD

    RightIntake(new Pose2d(1.227, 1.048, Rotation2d.fromDegrees(55))), // Right intake station
    LeftIntake(new Pose2d(1.227, 6.983, Rotation2d.fromDegrees(-55))), // Left intake station

    PROCESSOR(new Pose2d(6.0, 0.6, Rotation2d.fromDegrees(-90))),

    NET(new Pose2d(7.7, 6.0, Rotation2d.fromDegrees(0)));

    public Pose2d value;

    private ScoringLocations(Pose2d value) {
      this.value = value;
    }

    private Pose2d middle(ScoringLocations other) {
      return value.interpolate(other.value, 0.5);
    }
  }

  public enum ScoringLocationsLeft {
    A(ScoringLocations.A.value), C(ScoringLocations.C.value), E(ScoringLocations.E.value), G(
        ScoringLocations.G.value), I(ScoringLocations.I.value), K(ScoringLocations.K.value);

    public Pose2d value;

    private ScoringLocationsLeft(Pose2d value) {
      this.value = value;
    }
  }

  public enum ScoringLocationsRight {
    B(ScoringLocations.B.value), D(ScoringLocations.D.value), F(ScoringLocations.F.value), H(
        ScoringLocations.H.value), J(ScoringLocations.J.value), L(ScoringLocations.L.value);

    public Pose2d value;

    private ScoringLocationsRight(Pose2d value) {
      this.value = value;
    }
  }

  public enum ScoringLocationsCenter {
    /** AB */
    AB(ScoringLocations.A.middle(ScoringLocations.B)),
    /** CD */
    CD(ScoringLocations.C.middle(ScoringLocations.D)),
    /** EF */
    EF(ScoringLocations.E.middle(ScoringLocations.F)),
    /** GH */
    GH(ScoringLocations.G.middle(ScoringLocations.H)),
    /** IJ */
    IJ(ScoringLocations.I.middle(ScoringLocations.J)),
    /** KL */
    KL(ScoringLocations.K.middle(ScoringLocations.L));

    public Pose2d value;

    private ScoringLocationsCenter(Pose2d pose) {
      value = pose;
    }
  }

  public static class LedConstants {
    public static final int numLED = 133;
    public static final double flashSpeed = 0.75;
    public static final double strobeSpeed = 0.1;
    public static final double endgameWarning = 30;
    public static final double endgameAlert = 15;
    public static final int funnelOffset = 8; // 8
    public static final int elevatorOffset = 95; // 94
    public static final int funnelNumLED = 81; // 85
    public static final int elevatorNumLED = 40; // 40
    public static final int funnelOffset2 = 8; // 8
    public static final int elevatorOffset2 = 93; // 94
    public static final int funnelNumLED2 = elevatorOffset2 - funnelOffset2; // 85
    public static final int elevatorNumLED2 = 40; // 40
  }

  public static class FFConstants {
    public static final double k_bargeX = 8.774176;
    public static final double k_radius = 1.27;
    public static final double k_decceleration = 6.0;
  }

  public enum CoralLevel {
    L1, SecondaryL1, L2, L3, L4;

    public ElevatorState toElevatorState() {
      switch (this) {
        case L1:
          return ElevatorState.L1;
        case L2:
          return ElevatorState.L2;
        case L3:
          return ElevatorState.L3;
        case L4:
          return ElevatorState.L4;
        case SecondaryL1:
          return ElevatorState.SecondaryL1;
        default:
          return ElevatorState.Stow;
      }
    }
  }
}
