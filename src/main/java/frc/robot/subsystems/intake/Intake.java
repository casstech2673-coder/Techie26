// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public enum Goal {
    IDLE,
    INTAKING,
    KICK,
    AGITATE,
    EJECT,
    /** Intake roller only — no hopper, no arm movement. Operator LT override. */
    ROLLER_ONLY,
    /** Operator Y held: slowly drives arm toward kArmMaxDeg. */
    MANUAL_ARM_UP,
    /** Operator A held: slowly drives arm toward 0°. */
    MANUAL_ARM_DOWN
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private Goal currentGoal = Goal.IDLE;

  private final Timer agitateTimer = new Timer();
  private boolean agitateArmUp = false;

  // Tracks commanded arm angle for incremental manual control.
  private double manualArmAngleDeg = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
    agitateTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    switch (currentGoal) {
      case IDLE -> {
        io.stopAll();
        io.setArmPosition(IntakeConstants.kArmIdleDeg);
        agitateTimer.stop();
        agitateTimer.reset();
        agitateArmUp = false;
      }
      case INTAKING -> {
        io.setIntakeOutput(IntakeConstants.kIntakeSpeed);
        io.setHopperOutput(IntakeConstants.kHopperFeedSpeed);
        io.setArmPosition(IntakeConstants.kArmIdleDeg);
      }
      case KICK -> {
        io.setHopperOutput(IntakeConstants.kHopperKickSpeed);
        io.setIntakeOutput(0.0);
        io.setArmPosition(IntakeConstants.kArmIdleDeg);
      }
      case AGITATE -> executeAgitate();
      case EJECT -> {
        io.setIntakeOutput(IntakeConstants.kEjectSpeed);
        io.setHopperOutput(IntakeConstants.kHopperEjectSpeed);
        io.setArmPosition(IntakeConstants.kArmIdleDeg);
      }
      case ROLLER_ONLY -> {
        io.setIntakeOutput(IntakeConstants.kIntakeSpeed);
        io.setHopperOutput(0.0);
        io.setArmPosition(IntakeConstants.kArmIdleDeg);
      }
      case MANUAL_ARM_UP -> {
        manualArmAngleDeg =
            MathUtil.clamp(
                manualArmAngleDeg + IntakeConstants.kArmManualDegPerSec * 0.020,
                0.0,
                IntakeConstants.kArmMaxDeg);
        io.setArmPosition(manualArmAngleDeg);
        io.setIntakeOutput(0.0);
        io.setHopperOutput(0.0);
      }
      case MANUAL_ARM_DOWN -> {
        manualArmAngleDeg =
            MathUtil.clamp(
                manualArmAngleDeg - IntakeConstants.kArmManualDegPerSec * 0.020,
                0.0,
                IntakeConstants.kArmMaxDeg);
        io.setArmPosition(manualArmAngleDeg);
        io.setIntakeOutput(0.0);
        io.setHopperOutput(0.0);
      }
    }

    Logger.recordOutput("Intake/AgitateArmUp", agitateArmUp);
    Logger.recordOutput("Intake/ArmPositionDeg", inputs.armPositionDeg);
  }

  /** Arm oscillates between down and up to shake loose jams; hopper runs at slow speed. */
  private void executeAgitate() {
    if (!agitateArmUp) {
      io.setArmPosition(IntakeConstants.kArmAgitateDownDeg);
      io.setHopperOutput(IntakeConstants.kHopperSlowSpeed);
      io.setIntakeOutput(0.0);
      if (agitateTimer.hasElapsed(IntakeConstants.kArmAgitateDownDurationSec)) {
        agitateArmUp = true;
        agitateTimer.reset();
      }
    } else {
      io.setArmPosition(IntakeConstants.kArmAgitateUpDeg);
      io.setHopperOutput(0.0);
      io.setIntakeOutput(0.0);
      if (agitateTimer.hasElapsed(IntakeConstants.kArmAgitateUpDurationSec)) {
        agitateArmUp = false;
        agitateTimer.reset();
      }
    }
  }

  public void setGoal(Goal goal) {
    if (goal != currentGoal) {
      if (goal != Goal.AGITATE) {
        agitateTimer.reset();
        agitateArmUp = false;
      }
      // Sync manual arm setpoint to actual position on entry to avoid jumps.
      if (goal == Goal.MANUAL_ARM_UP || goal == Goal.MANUAL_ARM_DOWN) {
        manualArmAngleDeg = inputs.armPositionDeg;
      }
    }
    this.currentGoal = goal;
  }

  @AutoLogOutput(key = "Intake/Goal")
  public Goal getGoal() {
    return currentGoal;
  }

  /** Current arm pivot position in degrees (0° = fully down). */
  @AutoLogOutput(key = "Intake/ArmPositionDeg")
  public double getArmPositionDeg() {
    return inputs.armPositionDeg;
  }
}
