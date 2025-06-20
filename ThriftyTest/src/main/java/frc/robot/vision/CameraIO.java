package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Transform3d;

public interface CameraIO {
  void updateInputs(CameraIOInputs inputs);

  public class CameraIOInputs {
    public boolean connected = true;
    public List<PhotonPipelineResult> unreadResults = new ArrayList<>();
  }

  String getName();
  Transform3d getRobotToCamera();
}
