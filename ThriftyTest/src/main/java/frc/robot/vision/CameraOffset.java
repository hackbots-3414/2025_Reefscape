package frc.robot.vision;

import java.util.Map;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraOffset (
    String source,
    Map<Integer, Transform3d> offsets
) {}
