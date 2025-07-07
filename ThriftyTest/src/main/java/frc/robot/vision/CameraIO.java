package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;

public interface CameraIO {
  void updateInputs(CameraIOInputs inputs);

  public class CameraIOInputs {
    public boolean connected = true;
    public List<PhotonPipelineResult> unreadResults = new ArrayList<>();
  }
}
