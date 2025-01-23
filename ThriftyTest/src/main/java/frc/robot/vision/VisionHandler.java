package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionHandler implements AutoCloseable {
    private Logger m_logger = LoggerFactory.getLogger(VisionHandler.class);
    private CommandSwerveDrivetrain m_drivetrain;
    private Notifier m_notifier;
    private List<SingleInputPoseEstimator> m_estimators = new ArrayList<>();

    private VisionSystemSim m_visionSim = new VisionSystemSim("main");
    private SimCameraProperties m_simProps = new SimCameraProperties();

    private Field2d m_field;

    public VisionHandler(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        try {
            setupAprilTagField();
        } catch (IOException e) {
            m_logger.error("could not load april tag resource file");
            System.exit(1);
        }
        m_simProps.setCalibration(320, 240, new Rotation2d(Degrees.of(70)));
        m_simProps.setAvgLatencyMs(18);
        m_simProps.setLatencyStdDevMs(5);
        m_simProps.setCalibError(0.03, 0.02);
        setupCameras();
        m_notifier = new Notifier(() -> {
            for (Runnable estimator : m_estimators) {
                estimator.run();
            }
            m_visionSim.update(drivetrain.getPose());
        });
        m_visionSim.addVisionTargets(
            new VisionTargetSim(
                new Pose3d(1, 1, 3, new Rotation3d()),
                new TargetModel(2, 2, 2)
            )
        );
        m_field = m_visionSim.getDebugField();
        SmartDashboard.putData("April Tag Debug Field", m_field);
    }

    public void startThread() {
        m_notifier.startPeriodic(VisionConstants.k_periodic);
    }

    private void setupCameras() {
        for (Map.Entry<String, Transform3d> entry : VisionConstants.cameras.entrySet()) {
            // it's easier to read this way:
            String cameraName = entry.getKey();
            Transform3d robotToCamera = entry.getValue();
            // initialze both real and simulated cameras
            PhotonCamera realCamera = new PhotonCamera(cameraName);
            PhotonCameraSim simCamera = new PhotonCameraSim(realCamera, m_simProps);
            // sim camera required a little more configuration
            m_visionSim.addCamera(simCamera, robotToCamera);
            if (Robot.isSimulation()) {
                // This is highly computer intensive and not intended for real
                // competition use. it will boink the roborio's cpu :(
                simCamera.enableDrawWireframe(true);
            }
            SingleInputPoseEstimator estimator = new SingleInputPoseEstimator(
                realCamera,
                robotToCamera,
                (TimestampedPoseEstimate estimate) -> {
                    m_field.getObject(cameraName).setPose(estimate.pose());
                    m_drivetrain.addPoseEstimate(estimate);
                }
            );
            m_estimators.add(estimator);
        }
    }

    public void setupAprilTagField() throws IOException {
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(
            AprilTagFields.k2025Reefscape.m_resourceFile
        );
        m_visionSim.addAprilTags(tagLayout);
    }

    public void close() {
        m_notifier.close();
    }
}
