package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record TimestampedPoseEstimate (
    Pose2d pose,
    String source,
    double timestamp,
    Matrix<N3, N1> stdDevs
) {}
