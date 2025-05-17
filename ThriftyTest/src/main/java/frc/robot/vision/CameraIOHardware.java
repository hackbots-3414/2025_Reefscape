package frc.robot.vision;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraIOHardware implements CameraIO {
  private final PhotonCamera m_camera;
  private final String m_name;
  private final Transform3d m_robotToCamera;

  public CameraIOHardware(String name, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(name);
    m_name = name;
    m_robotToCamera = robotToCamera;
  }

  public void updateInputs(CameraIOInputs inputs) {
    inputs.connected = m_camera.isConnected();
    inputs.unreadResults = m_camera.getAllUnreadResults();
  }

  public String getName() {
    return m_name;
  }

  public Transform3d getRobotToCamera() {
    return m_robotToCamera;
  }
}
