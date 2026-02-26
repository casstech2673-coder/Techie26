// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware-abstraction interface for a single vision camera.
 *
 * <p>Each camera gets its own VisionIO implementation: - {@link VisionIOLumaP1} → Luma P1 (2× used
 * for global pose estimation) - {@link VisionIOLimelight4} → Limelight 4 (mounted on turret, direct
 * target lock) - {@code VisionIO() {}} → Replay mode (no-ops; inputs come from log)
 *
 * <p>The {@link Vision} subsystem aggregates data from all camera instances into a unified {@link
 * Vision.TargetResult} and chooses the best source per control cycle.
 */
public interface VisionIO {

  // ── Logged Inputs ──────────────────────────────────────────────────────────
  @AutoLog
  class VisionIOInputs {

    // ── Target Detection ──────────────────────────────────────────────────────

    /** True when this camera currently sees at least one AprilTag / game piece. */
    public boolean targetInSight = false;

    /**
     * Horizontal angle offset from camera boresight to the primary target (degrees). Limelight:
     * this is the "tx" value (negative = target is left of center). Luma P1: not meaningful for
     * global pose; leave at 0.0.
     */
    public double txDeg = 0.0;

    /**
     * Vertical angle offset from camera boresight to the primary target (degrees). Limelight: this
     * is the "ty" value.
     */
    public double tyDeg = 0.0;

    /**
     * Target area as a percentage of image area (0-100%). Used as a rough distance proxy and
     * confidence filter. A very small target area (e.g., < 0.5%) suggests the target is far away.
     */
    public double targetAreaPercent = 0.0;

    // ── Global Pose Estimation (Luma P1 / AprilTag megaTag) ──────────────────

    /**
     * True when this camera has computed a valid global field pose. Requires seeing at least one
     * AprilTag with low ambiguity.
     */
    public boolean hasPoseEstimate = false;

    /**
     * The estimated robot pose on the field (from multi-tag AprilTag solve). Valid only when {@link
     * #hasPoseEstimate} is true.
     */
    public Pose2d estimatedPose = new Pose2d();

    /**
     * Timestamp of the camera exposure that produced {@link #estimatedPose}. CRITICAL: Used to add
     * the measurement to the drive's pose estimator at the correct historical timestamp
     * (compensates for vision latency). In FPGA seconds (Timer.getFPGATimestamp() format).
     */
    public double timestampSeconds = 0.0;

    /**
     * Pose ambiguity from the AprilTag PnP solve (0.0 = perfect, 1.0 = ambiguous). Filter out poses
     * with ambiguity > 0.2 to avoid bad odometry updates. Only meaningful when using a single tag;
     * multi-tag solves have near-zero ambiguity.
     */
    public double poseAmbiguity = 1.0;

    /**
     * Distance (in meters) from the camera to the nearest visible AprilTag. Closer tags give more
     * accurate pose estimates.
     */
    public double nearestTagDistanceMeters = Double.MAX_VALUE;

    /** Number of AprilTags visible to this camera this loop. More tags = more accurate pose. */
    public int numTagsVisible = 0;

    /**
     * Camera pipeline latency in milliseconds. Add this to frame capture timestamp when feeding
     * pose estimator.
     */
    public double latencyMs = 0.0;

    /** True when the camera is connected and sending data. */
    public boolean cameraConnected = false;
  }

  // ── Default (no-op) implementations ───────────────────────────────────────

  /** Refresh inputs. Called once per loop by Vision.periodic(). */
  default void updateInputs(VisionIOInputs inputs) {}
}
