// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem — aggregates all cameras and produces a single {@link TargetResult} per loop.
 *
 * <h2>Turret aiming priority</h2>
 *
 * <ol>
 *   <li><b>TARGET_LOCK</b> — Limelight 4 (on turret) sees the hub directly via MegaTag2.
 *       Turret-relative bearing = current turret angle + Limelight txDeg. This is the most accurate
 *       mode because it closes the loop around the physical target, independent of robot pose.
 *       Field heading of the turret = turretAngle + robotHeading; the target correction is purely
 *       turret-relative (txDeg), so robot heading cancels out.
 *   <li><b>FIELD_TRACKING</b> — Limelight is lost but at least one Luma P1 camera has a valid
 *       global pose. We compute the exact turret pivot position on the field (accounting for the
 *       robot–turret offset), then aim the turret at the hub using the fused odometry pose.
 *   <li><b>DEAD_RECKONING</b> — No vision. The last known target angle is held so the turret does
 *       not snap to 0° unexpectedly.
 * </ol>
 */
public class Vision extends SubsystemBase {

  public enum VisionMode {
    TARGET_LOCK,
    FIELD_TRACKING,
    DEAD_RECKONING
  }

  /**
   * Result of one vision cycle.
   *
   * @param mode Which mode produced this result.
   * @param hasTarget True when the result contains a valid target bearing.
   * @param targetAngleDeg Desired turret angle (robot-relative degrees) to face the hub.
   * @param distanceToTargetMeters Estimated distance from turret pivot to hub (meters).
   * @param latencyMs Frame latency (ms), only meaningful in TARGET_LOCK mode.
   * @param poseAmbiguity PnP ambiguity (0 = perfect; only used in FIELD_TRACKING).
   */
  public record TargetResult(
      VisionMode mode,
      boolean hasTarget,
      double targetAngleDeg,
      double distanceToTargetMeters,
      double latencyMs,
      double poseAmbiguity) {}

  // ── Camera constants ──────────────────────────────────────────────────────
  // Distance estimate from Limelight tyDeg (trigonometry).
  // *** Update these to match the physical Limelight 4 mounting position. ***
  private static final double kTargetHeightMeters = 2.05; // hub target height above floor
  private static final double kCameraHeightMeters = 0.60; // Limelight lens height above floor
  private static final double kCameraMountAngleDeg = 30.0; // Limelight pitch (up from horizontal)

  // Confidence thresholds for pose-based (Luma P1) measurements
  private static final double kMaxAmbiguity = 0.2;
  private static final double kMaxTagDistMeters = 5.0;
  private static final double kMinTargetArea = 0.1;

  // ── Camera array ─────────────────────────────────────────────────────────
  private final VisionIO[] cameras;
  private final VisionIOInputsAutoLogged[] inputs;

  // ── External suppliers (wired by RobotContainer) ─────────────────────────
  @FunctionalInterface
  public interface PoseConsumer {
    void accept(Pose2d pose, double timestampSeconds, double[] stdDevs);
  }

  private PoseConsumer poseConsumer = (pose, ts, stdDevs) -> {};
  private DoubleSupplier turretAngleSupplier = () -> 0.0;
  private Supplier<Pose2d> robotPoseSupplier = null; // fused odometry pose (set by RobotContainer)

  // ── Hub position (updated by Superstructure each loop via setTargetFieldPosition) ──────────
  // Defaults to the blue hub; Superstructure calls setTargetFieldPosition() with the
  // alliance-correct value from FieldConstants.getHubForAlliance() each periodic loop.
  private Translation2d targetFieldPosition = FieldConstants.kBlueHubCenter;

  // ── State ─────────────────────────────────────────────────────────────────
  private VisionMode activeMode = VisionMode.DEAD_RECKONING;
  private TargetResult latestResult =
      new TargetResult(VisionMode.DEAD_RECKONING, false, 0.0, 5.0, 0.0, 1.0);
  private double lastKnownTargetAngle = 0.0;

  // ── Constructor ───────────────────────────────────────────────────────────
  public Vision(VisionIO... cameras) {
    this.cameras = cameras;
    this.inputs = new VisionIOInputsAutoLogged[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  // ── Wiring (called from RobotContainer) ──────────────────────────────────

  /** Receives valid global pose estimates and feeds them to the drive's pose estimator. */
  public void setPoseConsumer(PoseConsumer consumer) {
    this.poseConsumer = consumer;
  }

  /** Supplies the current turret encoder angle (robot-relative degrees). */
  public void setTurretAngleSupplier(DoubleSupplier supplier) {
    this.turretAngleSupplier = supplier;
  }

  /**
   * Supplies the fused robot pose from the drive subsystem's pose estimator.
   *
   * <p>Used in FIELD_TRACKING mode to compute the exact turret pivot position and aim at the hub.
   * Without this, Vision falls back to using the raw camera pose (less stable).
   */
  public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    this.robotPoseSupplier = supplier;
  }

  /**
   * Sets the field position of the scoring target.
   *
   * <p>Called by Superstructure each loop with {@link FieldConstants#getHubForAlliance()} so the
   * correct hub is targeted for the current alliance.
   */
  public void setTargetFieldPosition(Translation2d position) {
    this.targetFieldPosition = position;
  }

  // ── Periodic ─────────────────────────────────────────────────────────────
  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Feed valid global poses from Luma P1 cameras (indices 0 and 1) to the drive estimator.
    for (int i = 0; i < Math.min(2, cameras.length); i++) {
      VisionIOInputsAutoLogged cam = inputs[i];
      if (!cam.hasPoseEstimate) continue;
      if (cam.poseAmbiguity > kMaxAmbiguity) continue;
      if (cam.nearestTagDistanceMeters > kMaxTagDistMeters) continue;

      double distFactor = cam.nearestTagDistanceMeters;
      double tagFactor = 1.0 / Math.max(1, cam.numTagsVisible);
      double xyStdDev = 0.5 * distFactor * tagFactor;
      double thetaStdDev = 1.0 * distFactor * tagFactor;
      poseConsumer.accept(
          cam.estimatedPose, cam.timestampSeconds, new double[] {xyStdDev, xyStdDev, thetaStdDev});
    }

    // Reference aliases for the three camera slots.
    VisionIOInputsAutoLogged ll = inputs.length > 2 ? inputs[2] : null; // Limelight 4
    VisionIOInputsAutoLogged lumaA = inputs.length > 0 ? inputs[0] : null; // Luma P1 A
    VisionIOInputsAutoLogged lumaB = inputs.length > 1 ? inputs[1] : null; // Luma P1 B

    // ── Priority 1: TARGET_LOCK via Limelight MegaTag2 ──────────────────────
    // The Limelight is mounted on the turret. Its txDeg is the angular error from the
    // turret's boresight to the target. To centre the target:
    //   new turret angle = currentTurretAngle + txDeg   (robot-relative)
    // This is independent of robot heading — it only requires the turret encoder.
    if (ll != null
        && ll.targetInSight
        && ll.targetAreaPercent >= kMinTargetArea
        && ll.cameraConnected) {

      double turretAngle = turretAngleSupplier.getAsDouble();
      double targetAngle = turretAngle + ll.txDeg; // robot-relative setpoint to centre hub
      double distanceM = estimateDistanceFromTy(ll.tyDeg);
      lastKnownTargetAngle = targetAngle;
      activeMode = VisionMode.TARGET_LOCK;
      latestResult =
          new TargetResult(VisionMode.TARGET_LOCK, true, targetAngle, distanceM, ll.latencyMs, 0.0);

      // ── Priority 2: FIELD_TRACKING via global pose (Luma P1 / fused odometry) ──
      // No direct sight-line to hub. Compute the turret pivot's position on the field,
      // then derive the angle from it to the hub.
      //
      //   1. Get the best available robot pose (fused odometry preferred; raw camera fallback).
      //   2. turretCenter = robotCenter + rotate(turretOffset, robotHeading)
      //   3. toHub = hubPosition − turretCenter
      //   4. fieldBearing = atan2(toHub.y, toHub.x)
      //   5. turretAngle = fieldBearing − robotHeading   (convert field→robot frame)
    } else if (hasBestPose(lumaA, lumaB)) {

      Pose2d robotPose = getBestRobotPose(lumaA, lumaB);
      Translation2d turretCenter =
          robotPose
              .getTranslation()
              .plus(TurretConstants.kTurretOffsetFromRobotCenter.rotateBy(robotPose.getRotation()));
      Translation2d toTarget = targetFieldPosition.minus(turretCenter);
      double fieldBearingDeg = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
      double turretTargetAngle = fieldBearingDeg - robotPose.getRotation().getDegrees();
      double distanceM = toTarget.getNorm();
      double ambiguity = bestAmbiguity(lumaA, lumaB);
      lastKnownTargetAngle = turretTargetAngle;
      activeMode = VisionMode.FIELD_TRACKING;
      latestResult =
          new TargetResult(
              VisionMode.FIELD_TRACKING, true, turretTargetAngle, distanceM, 0.0, ambiguity);

      // ── Priority 3: DEAD_RECKONING — hold last known angle ──────────────────
    } else {
      activeMode = VisionMode.DEAD_RECKONING;
      latestResult =
          new TargetResult(VisionMode.DEAD_RECKONING, false, lastKnownTargetAngle, 5.0, 0.0, 1.0);
    }

    Logger.recordOutput("Vision/ActiveMode", activeMode.name());
    Logger.recordOutput("Vision/HasTarget", latestResult.hasTarget());
    Logger.recordOutput("Vision/TargetAngleDeg", latestResult.targetAngleDeg());
    Logger.recordOutput("Vision/DistanceToTargetMeters", latestResult.distanceToTargetMeters());
  }

  // ── Public Accessors ─────────────────────────────────────────────────────

  public TargetResult getTargetResult() {
    return latestResult;
  }

  public VisionMode getActiveMode() {
    return activeMode;
  }

  public Translation2d getTargetFieldPosition() {
    return targetFieldPosition;
  }

  // ── Private Helpers ───────────────────────────────────────────────────────

  /** Trigonometric distance estimate from the Limelight's vertical angle (tyDeg). */
  private double estimateDistanceFromTy(double tyDeg) {
    double deltaH = kTargetHeightMeters - kCameraHeightMeters;
    double totalAngleRad = Math.toRadians(kCameraMountAngleDeg + tyDeg);
    if (Math.abs(totalAngleRad) < 0.01) return 5.0;
    return deltaH / Math.tan(totalAngleRad);
  }

  /**
   * Returns the best robot pose to use for FIELD_TRACKING.
   *
   * <p>Prefers the fused odometry pose (supplied via {@link #setRobotPoseSupplier}) because it
   * blends all sensor inputs and is smoother than a raw single-camera estimate. Falls back to the
   * lowest-ambiguity Luma P1 pose if no fused pose supplier is wired.
   */
  private Pose2d getBestRobotPose(VisionIOInputsAutoLogged a, VisionIOInputsAutoLogged b) {
    if (robotPoseSupplier != null) {
      Pose2d fused = robotPoseSupplier.get();
      if (fused != null) return fused;
    }
    return pickBestCameraPose(a, b);
  }

  private boolean hasBestPose(VisionIOInputsAutoLogged a, VisionIOInputsAutoLogged b) {
    boolean aValid = a != null && a.hasPoseEstimate && a.poseAmbiguity <= kMaxAmbiguity;
    boolean bValid = b != null && b.hasPoseEstimate && b.poseAmbiguity <= kMaxAmbiguity;
    // Accept if either camera is valid, OR if fused odometry is available as fallback
    return aValid || bValid || robotPoseSupplier != null;
  }

  private Pose2d pickBestCameraPose(VisionIOInputsAutoLogged a, VisionIOInputsAutoLogged b) {
    boolean aValid = a != null && a.hasPoseEstimate && a.poseAmbiguity <= kMaxAmbiguity;
    boolean bValid = b != null && b.hasPoseEstimate && b.poseAmbiguity <= kMaxAmbiguity;
    if (!aValid) return b.estimatedPose;
    if (!bValid) return a.estimatedPose;
    return a.poseAmbiguity <= b.poseAmbiguity ? a.estimatedPose : b.estimatedPose;
  }

  private double bestAmbiguity(VisionIOInputsAutoLogged a, VisionIOInputsAutoLogged b) {
    double aAmb = (a != null && a.hasPoseEstimate) ? a.poseAmbiguity : 1.0;
    double bAmb = (b != null && b.hasPoseEstimate) ? b.poseAmbiguity : 1.0;
    return Math.min(aAmb, bAmb);
  }
}
