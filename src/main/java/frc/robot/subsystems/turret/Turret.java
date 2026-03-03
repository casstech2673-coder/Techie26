// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {

  public enum Goal {
    HOMING,
    TRACKING,
    HOLD,
    STOW
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private Goal currentGoal = Goal.STOW;
  private double requestedTargetAngleDeg = 0.0;
  private double resolvedTargetAngleDeg = 0.0;
  private boolean isWrappingFlag = false;
  private boolean hasHomed = false;

  private static final LoggedNetworkNumber tunableKp =
      new LoggedNetworkNumber("Turret/kP", TurretConstants.kTurretKp);
  private static final LoggedNetworkNumber tunableKd =
      new LoggedNetworkNumber("Turret/kD", TurretConstants.kTurretKd);

  // Track last-applied PID values to avoid re-applying unchanged gains every loop
  private double lastKp = TurretConstants.kTurretKp;
  private double lastKd = TurretConstants.kTurretKd;

  public Turret(TurretIO io) {
    this.io = io;
    // Software-zero the encoder to the known startup position instead of a homing sequence.
    // The robot is always placed at the same orientation before enable.
    io.seedStartupPosition(TurretConstants.kStartupAngleDeg);
    hasHomed = true;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Apply PID gains to hardware only when tunable values change
    double kp = tunableKp.get();
    double kd = tunableKd.get();
    if (kp != lastKp || kd != lastKd) {
      io.setPIDGains(kp, kd);
      lastKp = kp;
      lastKd = kd;
    }

    switch (currentGoal) {
      case HOMING -> executeHoming();
      case TRACKING -> {
        if (!hasHomed) io.holdPosition();
        else executeTracking();
      }
      case HOLD -> io.holdPosition();
      case STOW -> io.setTargetAngle(resolveWithWrapping(0.0));
    }

    // "Turret/IsWrapping", "Turret/IsAligned", "Turret/CurrentAngleDeg", "Turret/HasHomed" are
    // logged via @AutoLogOutput on their getter methods — do NOT repeat them here to avoid
    // duplicate-key conflicts in AdvantageKit replay.
    Logger.recordOutput("Turret/Goal", currentGoal.name());
    Logger.recordOutput("Turret/RequestedAngleDeg", requestedTargetAngleDeg);
    Logger.recordOutput("Turret/ResolvedAngleDeg", resolvedTargetAngleDeg);
  }

  private void executeHoming() {
    if (inputs.atForwardLimit) {
      io.zeroEncoder();
      io.setVoltage(1.0);
      hasHomed = true;
      currentGoal = Goal.STOW;
    } else {
      io.setVoltage(TurretConstants.kHomingVoltage);
    }
  }

  private void executeTracking() {
    isWrappingFlag = false;
    resolvedTargetAngleDeg = resolveWithWrapping(requestedTargetAngleDeg);
    io.setTargetAngle(resolvedTargetAngleDeg);
  }

  private double resolveWithWrapping(double rawDeg) {
    double normalized = MathUtil.inputModulus(rawDeg, -180.0, 180.0);

    if (normalized >= TurretConstants.kSoftMinAngleDeg
        && normalized <= TurretConstants.kSoftMaxAngleDeg) {
      return normalized;
    }

    double wrapped =
        normalized > TurretConstants.kSoftMaxAngleDeg ? normalized - 360.0 : normalized + 360.0;

    if (wrapped < TurretConstants.kSoftMinAngleDeg || wrapped > TurretConstants.kSoftMaxAngleDeg) {
      return MathUtil.clamp(
          normalized, TurretConstants.kSoftMinAngleDeg, TurretConstants.kSoftMaxAngleDeg);
    }

    double current = inputs.currentAngleDeg;
    double distDirect =
        Math.abs(
            current
                - MathUtil.clamp(
                    normalized,
                    TurretConstants.kSoftMinAngleDeg,
                    TurretConstants.kSoftMaxAngleDeg));
    double distWrapped = Math.abs(current - wrapped);

    if (distWrapped < distDirect) {
      isWrappingFlag = true;
      return wrapped;
    }
    return MathUtil.clamp(
        normalized, TurretConstants.kSoftMinAngleDeg, TurretConstants.kSoftMaxAngleDeg);
  }

  public void setGoal(Goal goal) {
    this.currentGoal = goal;
  }

  public void setTargetAngle(double angleDeg) {
    this.requestedTargetAngleDeg = angleDeg;
  }

  @AutoLogOutput(key = "Turret/IsAligned")
  public boolean isAligned() {
    return Math.abs(inputs.currentAngleDeg - resolvedTargetAngleDeg)
        <= TurretConstants.kAlignedToleranceDeg;
  }

  @AutoLogOutput(key = "Turret/IsWrapping")
  public boolean isWrapping() {
    return isWrappingFlag;
  }

  @AutoLogOutput(key = "Turret/CurrentAngleDeg")
  public double getCurrentAngleDeg() {
    return inputs.currentAngleDeg;
  }

  @AutoLogOutput(key = "Turret/HasHomed")
  public boolean hasHomed() {
    return hasHomed;
  }
}
