package frc.robot.vision.localization;

import static edu.wpi.first.units.Units.Milliseconds;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotObserver;
import frc.robot.vision.CameraIO;

public class CameraIOAprilTagSim implements CameraIO {
  private static boolean setupComplete;

  private static final VisionSystemSim simSystem = new VisionSystemSim("localization");

  private static final SimCameraProperties simProps = new SimCameraProperties();

  private static final Field2d simField = simSystem.getDebugField();

  private final PhotonCamera m_camera;
  private final PhotonCameraSim m_cameraSim;
  private final Transform3d m_robotToCamera;

  private final Supplier<Pose2d> m_poseSupplier;

  private final String m_name;

  public CameraIOAprilTagSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    setupSimProps();
    m_name = name;
    m_poseSupplier = poseSupplier;
    m_camera = new PhotonCamera(name);
    m_robotToCamera = robotToCamera;
    m_cameraSim = new PhotonCameraSim(m_camera, simProps);
    m_cameraSim.enableDrawWireframe(true);
    simSystem.addCamera(m_cameraSim, m_robotToCamera);
    SmartDashboard.putBoolean("Vision/" + m_name + " connected", true);
  }

  public void updateInputs(CameraIOInputs inputs) {
    inputs.connected = SmartDashboard.getBoolean("Vision/" + m_name + " connected", true);
    simSystem.update(m_poseSupplier.get());
    inputs.unreadResults = m_camera.getAllUnreadResults();
  }

  public String getName() {
    return m_camera.getName();
  }

  private static void setupSimProps() {
    if (setupComplete) {
      return;
    }
    setupComplete = true;
    simSystem.addAprilTags(LocalizationConstants.kTagLayout);
    simProps.setCalibration(
        LocalizationConstants.kResWidth,
        LocalizationConstants.kResHeight,
        LocalizationConstants.kFOV);
    simProps.setAvgLatencyMs(LocalizationConstants.kAvgLatency.in(Milliseconds));
    simProps.setLatencyStdDevMs(LocalizationConstants.kLatencyStdDev.in(Milliseconds));
    simProps.setCalibError(LocalizationConstants.kAvgErr, LocalizationConstants.kErrStdDevs);
    RobotObserver.setField(simField);
  }
}
