package frc.robot.vision;

import org.photonvision.PhotonCamera;

public class CameraIOHardware implements CameraIO {
  private final PhotonCamera m_camera;
  private final String m_name;

  public CameraIOHardware(String name) {
    m_camera = new PhotonCamera(name);
    m_name = name;
  }

  public void updateInputs(CameraIOInputs inputs) {
    inputs.connected = m_camera.isConnected();
    inputs.unreadResults = m_camera.getAllUnreadResults();
  }

  public String getName() {
    return m_name;
  }
}
