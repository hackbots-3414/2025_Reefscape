package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.json.simple.parser.ParseException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FFConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SimConstants;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import frc.robot.driveassist.ForceField;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.LoopTimer;
import frc.robot.utils.OnboardLogger;
import frc.robot.vision.localization.TimestampedPoseEstimate;
import frc.robot.vision.tracking.AlgaeTracker.ObjectTrackingStatus;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  @SuppressWarnings("unused")
  private final Logger m_logger = LoggerFactory.getLogger(CommandSwerveDrivetrain.class);
  private final OnboardLogger m_ologger;

  private LoopTimer m_timer;

  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private boolean m_aligned;

  private Trigger m_tippyTrigger = new Trigger(() -> false);
  private Trigger m_slowTrigger = new Trigger(() -> false);

  private boolean m_hasReceivedVisionUpdate;

  private FieldCentric m_teleopRequest = new FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
      .withDriveRequestType(DriveRequestType.Velocity);

  private RobotCentric m_trackingRequest = new RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity);

  private FieldCentricFacingAngle m_veloRequest = new FieldCentricFacingAngle()
      .withHeadingPID(DriveConstants.HeadingPID.kP, 0, 0)
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
      .withDriveRequestType(DriveRequestType.Velocity);

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /* last known object tracking input */
  private Optional<ObjectTrackingStatus> m_objectStatus = Optional.empty();

  private Pose2d m_estimatedPose = new Pose2d();

  private SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  private final ApplyRobotSpeeds autoRequest =
      new ApplyRobotSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Robot.isSimulation()) {
      startSimThread();
    }
    m_aligned = false;
    m_ologger = new OnboardLogger("Drivetrain");
    m_ologger.registerBoolean("Aligned", aligned());
    m_ologger.registerDouble("Velocity", this::getVelocity);
    m_ologger.registerPose("Estimated Pose", this::getPose);
    m_ologger.registerBoolean("Received Vision Update", () -> m_hasReceivedVisionUpdate);
    m_ologger.registerBoolean("Valid Object Estimate", seesAlgae());

    RobotObserver.setVelocitySupplier(this::getVelocity);
    RobotObserver.setNoElevatorZoneSupplier(dangerZone());
    RobotObserver.setReefReadySupplier(inReefZone());
    RobotObserver.setAlginedSupplier(aligned());

    m_timer = new LoopTimer("Drivetrain");
    initializePathPlanner();
  }

  public void initializePathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a
                           // starting pose)
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> driveWithChassisSpeeds(speeds),
          DriveConstants.kPathplannerHolonomicDriveController,
          config, // The robot configuration
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this); // Reference to this subsystem to set requirements

      initializeSetpointGenerator(config);

      PathPlannerLogging.setLogActivePathCallback(
          poses -> RobotObserver.getField().getObject("Pathfind Trajectory").setPoses(poses));
    } catch (IOException | ParseException e) {
      e.printStackTrace();
      System.exit(1);
    }
  }

  public void initializeSetpointGenerator(RobotConfig config) {
    setpointGenerator = new SwerveSetpointGenerator(config,
        Units.rotationsToRadians(DriveConstants.kMaxRotationalSpeed));

    // TODO: is this causing problems when the previous setpoint doesn't match the robot speeds?
    // We saw issues where it would try to drive the wrong way. This could easily be the cause if
    // I'm understanding this system correctly
    ChassisSpeeds currSpeeds = getRobotRelativeSpeeds();
    SwerveModuleState[] currStates = getState().ModuleStates;
    previousSetpoint =
        new SwerveSetpoint(currSpeeds, currStates, DriveFeedforwards.zeros(config.numModules));
  }

  private Translation2d getVelocityComponents() {
    double vx = getRobotRelativeSpeeds().vxMetersPerSecond;
    double vy = getRobotRelativeSpeeds().vyMetersPerSecond;
    Rotation2d theta = getPose().getRotation();
    return new Translation2d(vx, vy).rotateBy(theta);

  }

  private double getVelocity() {
    Translation2d velo = getVelocityComponents();
    return velo.getNorm();
  }

  public Pose2d getPose() {
    return m_estimatedPose;
  }

  /**
   * returns the current pose, with red side poses flipped
   */
  public Pose2d getBluePose() {
    return FieldUtils.getLocalPose(m_estimatedPose);
  }

  public void zeroPose() {
    setPose(new Pose2d());
  }

  public Command resetHeading() {
    // this uses Commands.runOnce() as opposed to this.runOnce() because we don't really want to add
    // requirements on this subsystem
    return Commands.runOnce(() -> setOperatorPerspectiveForward(getPose().getRotation()));
  }

  private void setPose(Pose2d pose) {
    resetPose(pose);
  }

  public Command setLocalHeading(Rotation2d heading) {
    return Commands.runOnce(() -> resetRotation(FieldUtils.getLocalRotation(heading)))
        .ignoringDisable(true);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return getState().Speeds;
  }

  private void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    previousSetpoint = setpointGenerator.generateSetpoint(
        previousSetpoint, // The previous setpoint
        speeds, // The desired target speeds
        0.02 // The loop time of the robot code, in seconds
    );

    setControl(autoRequest.withSpeeds(previousSetpoint.robotRelativeSpeeds()));
  }

  private void stop() {
    setControl(new SwerveRequest.SwerveDriveBrake());
  }

  @Override
  public void periodic() {
    m_timer.reset();
    m_estimatedPose = this.getState().Pose;

    RobotObserver.getField().setRobotPose(m_estimatedPose);

    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(allianceColor -> {
        setOperatorPerspectiveForward(
            allianceColor == Alliance.Red
                ? kRedAlliancePerspectiveRotation
                : kBlueAlliancePerspectiveRotation);
        m_hasAppliedOperatorPerspective = true;
      });
    }
    m_ologger.log();
    m_hasReceivedVisionUpdate = false;
    // Expire old algae tracking data
    if (m_objectStatus.isPresent() && m_objectStatus.get().isExpired()) {
      m_objectStatus = Optional.empty();
    }
    m_timer.log();
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(SimConstants.k_simPeriodic);
  }

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> setControl(m_translationCharacterization.withVolts(output)),
          null,
          this));

  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> setControl(m_steerCharacterization.withVolts(volts)),
          null,
          this));

  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> {
            /* output is actually radians per second, but SysId only supports "volts" */
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this));

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
  }

  public void addPoseEstimate(TimestampedPoseEstimate estimate) {
    m_hasReceivedVisionUpdate = true;
    // This should NOT run in simulation!
    if (Robot.isSimulation()) {
      return;
    }
    addVisionMeasurement(
        estimate.pose(),
        estimate.timestamp(),
        estimate.stdDevs());
  }

  public void addObjectTrackingData(ObjectTrackingStatus status) {
    m_objectStatus = Optional.of(status);
  }

  public Command sysIdQuasistaticTranslation(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineTranslation.quasistatic(direction);
  }

  public Command sysIdDynamicTranslation(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineTranslation.dynamic(direction);
  }

  public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineSteer.quasistatic(direction);
  }

  public Command sysIdDynamicSteer(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineSteer.dynamic(direction);
  }

  public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRotation.quasistatic(direction);
  }

  public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRotation.dynamic(direction);
  }

  private Trigger dangerZone() {
    return new Trigger(() -> {
      double distance = Math.abs(m_estimatedPose.getX() - FFConstants.k_bargeX);
      return distance < FFConstants.k_radius && !DriverStation.isAutonomous();
    });
  }

  public void setAligned(boolean aligned) {
    m_aligned = aligned;
  }

  public Trigger aligned() {
    return new Trigger(() -> m_aligned);
  }

  public Trigger inReefZone() {
    return new Trigger(() -> {
      double distanceToReef = getBluePose().getTranslation()
          .minus(FieldConstants.reefCenter)
          .getNorm();
      boolean inRange =
          (DriverStation.isAutonomous()) ? distanceToReef <= FieldConstants.kReefReadyAuton
              : distanceToReef <= FieldConstants.kReefReady;
      return inRange;
    });
  }

  public Trigger seesAlgae() {
    return new Trigger(() -> m_objectStatus.isPresent()
        && m_objectStatus.get().isWithin(DriveConstants.kObjectDistanceLimit));
  }

  /**
   * Drives the robot from given x, y, and rotatational velocity suppliers.
   */
  public Command teleopDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return run(() -> {
      double vx = x.getAsDouble() * DriveConstants.kMaxTeleopLinearSpeed;
      double vy = y.getAsDouble() * DriveConstants.kMaxTeleopLinearSpeed;
      Translation2d operatorRelative = new Translation2d(vx, vy);
      Translation2d fieldRelative = operatorRelative.rotateBy(getOperatorForwardDirection());
      double omega = rot.getAsDouble() * DriveConstants.kMaxTeleopAngularSpeed;

      if (Math.hypot(vx, vy) > 1e-3 || Math.abs(omega) > 1e-2) {
        setAligned(false);
      }
      if (RobotObserver.getFFEnabled()) {
        double newX = ForceField.calculate(
            fieldRelative.getX(),
            FFConstants.k_bargeX - m_estimatedPose.getX());
        fieldRelative = new Translation2d(newX, fieldRelative.getY());
      }
      setControl(m_teleopRequest
          .withVelocityX(fieldRelative.getX())
          .withVelocityY(fieldRelative.getY())
          .withRotationalRate(omega));
    });
  }

  public void setTippyTrigger(Trigger tippyTrigger) {
    m_tippyTrigger = tippyTrigger;
  }

  public void setSlowTrigger(Trigger slowTrigger) {
    m_slowTrigger = slowTrigger;
  }

  private AngularVelocity getMaxRotationalRate() {
    if (m_tippyTrigger.getAsBoolean()) {
      return DriveConstants.kMaxTippyAngularSpeed;
    } else {
      return DriveConstants.kMaxAngularSpeed;
    }
  }

  private LinearVelocity getMaxSpeed() {
    if (m_slowTrigger.getAsBoolean()) {
      return DriveConstants.kMaxTippySpeed;
    }
    return DriveConstants.kMaxLinearSpeed;
  }

  private void setVelocity(Transform2d goal) {
    double norm = goal.getTranslation().getNorm();
    double max = getMaxSpeed().in(MetersPerSecond);
    if (norm > max) {
      goal = goal.times(max / norm);
    }
    setControl(m_veloRequest
        .withVelocityX(goal.getX())
        .withVelocityY(goal.getY())
        .withTargetDirection(goal.getRotation())
        .withMaxAbsRotationalRate(getMaxRotationalRate()));
  }

  /**
   * Drives to a certain point on the field
   */
  public Command align(Autopilot autopilot, APTarget target) {
    return Commands.sequence(
        runOnce(() -> {
          RobotObserver.getField().getObject("reference").setPose(target.getReference());
          setAligned(false);
        }),
        run(() -> {
          Translation2d velocities = getVelocityComponents();
          Transform2d output = autopilot.calculate(m_estimatedPose, velocities, target);
          setVelocity(output);
        }))
        .until(() -> {
          return autopilot.atTarget(m_estimatedPose, target);
        })
        .finallyDo(this::stop)
        .finallyDo(interrupted -> setAligned(!interrupted))
        .finallyDo(() -> {
          RobotObserver.getField().getObject("reference").setPoses();
        });
  }

  public Command seedLocal(Pose2d pose) {
    return Commands.runOnce(() -> resetPose(FieldUtils.getLocalPose(pose)))
        .ignoringDisable(true);
  }

  public Command followObject() {
    return run(new Runnable() {
      private final PIDController thetaController = new PIDController(8, 0, 0);

      @Override
      public void run() {
        if (m_objectStatus.isEmpty()) {
          stop();
          return;
        }
        ObjectTrackingStatus status = m_objectStatus.get();
        if (status.pose().isEmpty()) {
          // Drive towards algae, based on camera location. NOT OPTIMAL.

          // We invert the yaw because the input is actually the angle is from the robot to the
          // target, so turning in that direction is good. (basically our input is reverse
          // coordinate system).
          double rotationalRate = thetaController.calculate(-status.yaw().getRadians(), 0);
          double speed = DriveConstants.kObjectTrackSpeed.in(MetersPerSecond);
          double vx = speed * status.yaw().getCos();
          double vy = speed * status.yaw().getSin();
          setControl(m_trackingRequest
              .withRotationalRate(rotationalRate)
              .withVelocityX(vx)
              .withVelocityY(vy));
          return;
        }
        // If we have a pose estimate, for algae, use Autopilot to go there.
        APTarget target = new APTarget(status.pose().get().toPose2d()
            .transformBy(new Transform2d(Translation2d.kZero, status.yaw()))
            .transformBy(DriveConstants.kAlgaeOffset));
        Transform2d output = DriveConstants.kTightAutopilot.calculate(
            m_estimatedPose,
            getVelocityComponents(),
            target);
        setVelocity(output);
      }
    }).onlyWhile(seesAlgae());
  }
}
