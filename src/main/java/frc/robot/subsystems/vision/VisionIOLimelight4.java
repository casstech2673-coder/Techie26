// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight4 implements VisionIO {

  private final String name;

  // Tracks the last time the Limelight sent valid pipeline data (non-zero latency).
  // Initialised to FPGA time at construction so cameraConnected starts true on boot
  // rather than staying false until the first target is seen.
  private double lastDataTime;

  public VisionIOLimelight4(String limelightName) {
    this.name = limelightName;
    this.lastDataTime = Timer.getFPGATimestamp();
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

    // A positive latency value means the camera processed a frame this loop.
    // This is a robust connectivity indicator regardless of whether a target is visible.
    if (tl > 0 || cl > 0) {
      lastDataTime = now;
    }
    inputs.cameraConnected = (now - lastDataTime) < 1.0; // 1 s timeout

    // Latency-corrected timestamp for when no MegaTag pose is available.
    inputs.timestampSeconds = now - (inputs.latencyMs / 1000.0);

    // MegaTag2 global pose estimate.
    // Gate on tagCount only — getTV() reflects the primary detection pipeline (may be a
    // game-piece pipeline) and must not be used to suppress AprilTag pose results.
    PoseEstimate megaTag = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (megaTag != null && megaTag.tagCount > 0) {
      inputs.hasPoseEstimate = true;
      inputs.estimatedPose = megaTag.pose;
      inputs.timestampSeconds = megaTag.timestampSeconds; // overwrite with accurate stamp
      inputs.nearestTagDistanceMeters = megaTag.avgTagDist;
      inputs.numTagsVisible = megaTag.tagCount;
      inputs.poseAmbiguity = 0.0; // MegaTag2 multi-tag solves are near-zero ambiguity
    } else {
      inputs.hasPoseEstimate = false;
      inputs.poseAmbiguity = 1.0;
    }
  }
}
