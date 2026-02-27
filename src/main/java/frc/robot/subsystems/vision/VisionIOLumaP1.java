// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOLumaP1 implements VisionIO {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  public VisionIOLumaP1(String cameraName, Transform3d robotToCamera) {
    camera = new PhotonCamera(cameraName);
    AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraConnected = camera.isConnected();
    if (!inputs.cameraConnected) return;

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) return;

    PhotonPipelineResult result = results.get(results.size() - 1);

    inputs.targetInSight = result.hasTargets();
    inputs.numTagsVisible = result.getTargets().size();
    inputs.timestampSeconds = result.getTimestampSeconds();
    inputs.latencyMs = (Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0;

    if (!inputs.targetInSight) {
      inputs.hasPoseEstimate = false;
      inputs.poseAmbiguity = 1.0;
      return;
    }

    Optional<EstimatedRobotPose> estimated = poseEstimator.estimateCoprocMultiTagPose(result);
    if (estimated.isPresent()) {
      EstimatedRobotPose pose = estimated.get();
      inputs.hasPoseEstimate = true;
      inputs.estimatedPose = pose.estimatedPose.toPose2d();
      inputs.poseAmbiguity =
          result.getTargets().size() == 1 ? result.getBestTarget().getPoseAmbiguity() : 0.0;
      inputs.nearestTagDistanceMeters =
          result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
      inputs.targetAreaPercent = result.getBestTarget().getArea();
    } else {
      inputs.hasPoseEstimate = false;
      inputs.poseAmbiguity = 1.0;
    }
  }
}
