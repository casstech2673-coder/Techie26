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

  @SuppressWarnings("unused")
  private static final LoggedNetworkNumber tunableKp =
      new LoggedNetworkNumber("Turret/kP", TurretConstants.kTurretKp);

  @SuppressWarnings("unused")
  private static final LoggedNetworkNumber tunableKd =
      new LoggedNetworkNumber("Turret/kD", TurretConstants.kTurretKd);

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    switch (currentGoal) {
      case HOMING -> executeHoming();
      case TRACKING -> {
        if (!hasHomed) io.holdPosition();
        else executeTracking();
      }
      case HOLD -> io.holdPosition();
      case STOW -> io.setTargetAngle(resolveWithWrapping(0.0));
    }

    Logger.recordOutput("Turret/Goal", currentGoal.name());
    Logger.recordOutput("Turret/RequestedAngleDeg", requestedTargetAngleDeg);
    Logger.recordOutput("Turret/ResolvedAngleDeg", resolvedTargetAngleDeg);
    Logger.recordOutput("Turret/IsWrapping", isWrappingFlag);
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
