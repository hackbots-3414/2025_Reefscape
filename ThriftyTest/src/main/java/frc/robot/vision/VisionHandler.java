package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import static edu.wpi.first.units.Units.Milliseconds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.RobotObserver;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionHandler implements AutoCloseable {
    private Logger m_logger = LoggerFactory.getLogger(VisionHandler.class);
    private CommandSwerveDrivetrain m_drivetrain;
    private final Notifier m_notifier;
    private List<SingleInputPoseEstimator> m_estimators = new ArrayList<>();

    private VisionSystemSim m_visionSim = new VisionSystemSim("main");
    private SimCameraProperties m_simProps = new SimCameraProperties();

    private final Field2d m_field;

    private boolean m_singleTag;

    private LogBuilder m_visionLogger;

    public VisionHandler(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        try {
            setupAprilTagField();
        } catch (IOException e) {
            m_logger.error("could not load april tag resource file");
            System.exit(1);
        }
        setupProps();
        setupCameras();
        m_notifier = new Notifier(this::updateEstimators);
        m_field = m_visionSim.getDebugField();
        RobotObserver.setField(m_field);
        m_singleTag = false;
    }

    private void setupAprilTagField() throws IOException {
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(
            AprilTagFields.k2025ReefscapeWelded.m_resourceFile
        );
        m_visionSim.addAprilTags(tagLayout);
    }

    /**
     * Sets up the simulated camera properties with values that should
     * reflect the real world situation.
     */
    private void setupProps() {
        m_simProps.setCalibration(
            VisionConstants.k_resWidth,
            VisionConstants.k_resHeight,
            VisionConstants.k_fov
        );
        m_simProps.setAvgLatencyMs(
            VisionConstants.k_avgLatency.in(Milliseconds)
        );
        m_simProps.setLatencyStdDevMs(
            VisionConstants.k_latencyStdDev.in(Milliseconds)
        );
        m_simProps.setCalibError(
            VisionConstants.k_avgErr,
            VisionConstants.k_errStdDev
        );
    }
    
    private void setupCameras() {
        for (Map.Entry<String, Transform3d> entry : VisionConstants.cameras.entrySet()) {
            // it's easier to read this way:
            String cameraName = entry.getKey();
            Transform3d robotToCamera = entry.getValue();
            // initialze both real and simulated cameras
            PhotonCamera realCamera = new PhotonCamera(cameraName);
            if (Robot.isSimulation()) {
                PhotonCameraSim simCamera = new PhotonCameraSim(realCamera, m_simProps);
                m_visionSim.addCamera(simCamera, robotToCamera);
            
                // This is somewhat intensive (especially the first one) so we only
                // enable if the robot is in simulation mode.
                simCamera.enableDrawWireframe(true);
            }
            // we always need to add a vision estimator
            SingleInputPoseEstimator estimator = new SingleInputPoseEstimator(
                realCamera,
                robotToCamera,
                this::addEstimate,
                this::getSingleTag
            );
            m_estimators.add(estimator);
        }
    }

    private void updateEstimators() {
        // logging
        m_visionLogger = new LogBuilder();
        // clear previous output from the estimators.
        m_field.getObject(VisionConstants.k_estimationName).setPoses();
        for (SingleInputPoseEstimator estimator : m_estimators) {
            estimator.run();
        }
        Pose2d currPose = m_drivetrain.getPose();
        m_visionSim.update(currPose);
        // finish logging
        m_visionLogger.setResult(currPose);
        m_visionLogger.log();
    }

    public void startThread() {
        m_notifier.startPeriodic(VisionConstants.k_periodic);
    }

    private void addEstimate(TimestampedPoseEstimate estimate) {
        List<Pose2d> poses = m_field.getObject(VisionConstants.k_estimationName)
            .getPoses();
        poses.add(estimate.pose());
        m_field.getObject(VisionConstants.k_estimationName).setPoses(poses);
        m_drivetrain.addPoseEstimate(estimate);
        // pose logging
        m_visionLogger.addEstimate(estimate);
    }

    private boolean getSingleTag() {
        return m_singleTag;
    }

    public void setMultitag() {
        m_singleTag = false;
        SmartDashboard.putBoolean("single tag", m_singleTag);
    }

    public void setSingleTag() {
        m_singleTag = true;
        SmartDashboard.putBoolean("single tag", m_singleTag);
    }

    @Override
    public void close() {
        m_notifier.close();
    }
}
