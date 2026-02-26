// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight4 implements VisionIO {

  private final String name;
  private double lastSeenTime = 0.0;

  public VisionIOLimelight4(String limelightName) {
    this.name = limelightName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double now = Timer.getFPGATimestamp();

    inputs.targetInSight = LimelightHelpers.getTV(name);
    inputs.txDeg = LimelightHelpers.getTX(name);
    inputs.tyDeg = LimelightHelpers.getTY(name);
    inputs.targetAreaPercent = LimelightHelpers.getTA(name);

    double tl = LimelightHelpers.getLatency_Pipeline(name);
    double cl = LimelightHelpers.getLatency_Capture(name);
    inputs.latencyMs = tl + cl;
    inputs.timestampSeconds = now - (inputs.latencyMs / 1000.0);

    if (inputs.targetInSight) lastSeenTime = now;
    inputs.cameraConnected = (now - lastSeenTime) < 0.5;

    PoseEstimate megaTag = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (megaTag != null && megaTag.tagCount > 0 && inputs.targetInSight) {
      inputs.hasPoseEstimate = true;
      inputs.estimatedPose = megaTag.pose;
      inputs.timestampSeconds = megaTag.timestampSeconds;
      inputs.nearestTagDistanceMeters = megaTag.avgTagDist;
      inputs.numTagsVisible = megaTag.tagCount;
      inputs.poseAmbiguity = 0.0;
    } else {
      inputs.hasPoseEstimate = false;
      inputs.poseAmbiguity = 1.0;
    }
  }
}
