// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.algaeTracking;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class AlgaeTracker {
    private final PhotonCamera camera;

    public AlgaeTracker(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    public Optional<AlgaeState> track() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.size() == 0) {
            return Optional.empty();
        }
        PhotonPipelineResult singleResult = results.get(0);
        if (!singleResult.hasTargets()) {
            return Optional.empty();
        } 
        PhotonTrackedTarget best = singleResult.getBestTarget();
        return Optional.of(new AlgaeState(Rotation2d.fromDegrees(-best.getYaw()), best.getArea()));
    }

    public record AlgaeState(
        Rotation2d rot,
        double size
    ) {}
}
