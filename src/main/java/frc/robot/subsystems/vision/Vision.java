// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  public enum VisionMode {
    TARGET_LOCK,
    FIELD_TRACKING,
    DEAD_RECKONING
  }

  public record TargetResult(
      VisionMode mode,
      boolean hasTarget,
      double targetAngleDeg,
      double distanceToTargetMeters,
      double latencyMs,
      double poseAmbiguity) {}

  // Camera height constants for distance estimation
  private static final double kTargetHeightMeters = 2.05;
  private static final double kCameraHeightMeters = 0.60;
  private static final double kCameraMountAngleDeg = 30.0;

  // Confidence thresholds
  private static final double kMaxAmbiguity = 0.2;
  private static final double kMaxTagDistMeters = 5.0;
  private static final double kMinTargetArea = 0.1;

  private final VisionIO[] cameras;
  private final VisionIOInputsAutoLogged[] inputs;

  private VisionMode activeMode = VisionMode.DEAD_RECKONING;
  private TargetResult latestResult =
      new TargetResult(VisionMode.DEAD_RECKONING, false, 0.0, 5.0, 0.0, 1.0);
  private double lastKnownTargetAngle = 0.0;

  @FunctionalInterface
  public interface PoseConsumer {
    void accept(Pose2d pose, double timestampSeconds, double[] stdDevs);
  }

  private PoseConsumer poseConsumer = (pose, ts, stdDevs) -> {};
  private DoubleSupplier turretAngleSupplier = () -> 0.0;
  private Translation2d targetFieldPosition = new Translation2d(0.0, 5.55);

  public Vision(VisionIO... cameras) {
    this.cameras = cameras;
    this.inputs = new VisionIOInputsAutoLogged[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  public void setPoseConsumer(PoseConsumer consumer) {
    this.poseConsumer = consumer;
  }

  public void setTurretAngleSupplier(DoubleSupplier supplier) {
    this.turretAngleSupplier = supplier;
  }

  public void setTargetFieldPosition(Translation2d position) {
    this.targetFieldPosition = position;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Feed valid global poses to drivetrain (Luma P1 cameras = indices 0 and 1)
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

    // Priority selection
    VisionIOInputsAutoLogged ll = inputs.length > 2 ? inputs[2] : null;
    VisionIOInputsAutoLogged lumaA = inputs.length > 0 ? inputs[0] : null;
    VisionIOInputsAutoLogged lumaB = inputs.length > 1 ? inputs[1] : null;

    if (ll != null
        && ll.targetInSight
        && ll.targetAreaPercent >= kMinTargetArea
        && ll.cameraConnected) {
      double turretAngle = turretAngleSupplier.getAsDouble();
      double targetAngle = turretAngle + ll.txDeg;
      double distanceM = estimateDistanceFromTy(ll.tyDeg);
      lastKnownTargetAngle = targetAngle;
      activeMode = VisionMode.TARGET_LOCK;
      latestResult =
          new TargetResult(VisionMode.TARGET_LOCK, true, targetAngle, distanceM, ll.latencyMs, 0.0);

    } else if (hasBestPose(lumaA, lumaB)) {
      Pose2d bestPose = pickBestPose(lumaA, lumaB);
      Translation2d toTarget = targetFieldPosition.minus(bestPose.getTranslation());
      double bearingDeg = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
      double turretTargetAngle = bearingDeg - bestPose.getRotation().getDegrees();
      double distanceM = toTarget.getNorm();
      double ambiguity = bestAmbiguity(lumaA, lumaB);
      lastKnownTargetAngle = turretTargetAngle;
      activeMode = VisionMode.FIELD_TRACKING;
      latestResult =
          new TargetResult(
              VisionMode.FIELD_TRACKING, true, turretTargetAngle, distanceM, 0.0, ambiguity);

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

  public TargetResult getTargetResult() {
    return latestResult;
  }

  public VisionMode getActiveMode() {
    return activeMode;
  }

  public Translation2d getTargetFieldPosition() {
    return targetFieldPosition;
  }

  private double estimateDistanceFromTy(double tyDeg) {
    double deltaH = kTargetHeightMeters - kCameraHeightMeters;
    double totalAngleRad = Math.toRadians(kCameraMountAngleDeg + tyDeg);
    if (Math.abs(totalAngleRad) < 0.01) return 5.0;
    return deltaH / Math.tan(totalAngleRad);
  }

  private boolean hasBestPose(VisionIOInputsAutoLogged a, VisionIOInputsAutoLogged b) {
    return (a != null && a.hasPoseEstimate && a.poseAmbiguity <= kMaxAmbiguity)
        || (b != null && b.hasPoseEstimate && b.poseAmbiguity <= kMaxAmbiguity);
  }

  private Pose2d pickBestPose(VisionIOInputsAutoLogged a, VisionIOInputsAutoLogged b) {
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
