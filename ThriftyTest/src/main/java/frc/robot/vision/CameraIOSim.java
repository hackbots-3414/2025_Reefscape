package frc.robot.vision;

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

public class CameraIOSim implements CameraIO {
  private static boolean setupComplete;

  private static final VisionSystemSim simSystem = new VisionSystemSim("main");

  private static final SimCameraProperties simProps = new SimCameraProperties();

  private static final Field2d simField = simSystem.getDebugField();

  private final PhotonCamera m_camera;
  private final PhotonCameraSim m_cameraSim;
  private final Transform3d m_robotToCamera;

  private final Supplier<Pose2d> m_poseSupplier;

  private final String m_name;

  public CameraIOSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
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

  public Transform3d getRobotToCamera() {
    return m_robotToCamera;
  }

  private static void setupSimProps() {
    if (setupComplete) {
      return;
    }
    setupComplete = true;
    simSystem.addAprilTags(VisionConstants.kTagLayout);
    simProps.setCalibration(
        VisionConstants.kResWidth,
        VisionConstants.kResHeight,
        VisionConstants.kFOV);
    simProps.setAvgLatencyMs(VisionConstants.kAvgLatency.in(Milliseconds));
    simProps.setLatencyStdDevMs(VisionConstants.kLatencyStdDev.in(Milliseconds));
    simProps.setCalibError(VisionConstants.kAvgErr, VisionConstants.kErrStdDevs);
    RobotObserver.setField(simField);
  }
}
