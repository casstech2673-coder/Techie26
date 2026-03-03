// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Constants for intake roller, hopper/kicker belt (single motor), and arm pivot. */
public final class IntakeConstants {

  // ── Motor CAN IDs ────────────────────────────────────────────────────────
  public static final int kIntakeMotorCanId = 40;
  public static final int kHopperMotorCanId = 41; // also serves as kicker (same motor)
  public static final int kArmMotorCanId = 43;

  // ── Duty Cycle Setpoints ────────────────────────────────────────────────
  public static final double kIntakeSpeed = 0.85;
  public static final double kEjectSpeed = -0.85;
  public static final double kHopperFeedSpeed = 0.60;
  public static final double kHopperSlowSpeed = 0.20;
  public static final double kHopperKickSpeed = 0.90;
  public static final double kHopperEjectSpeed = -0.90;

  // ── Arm Positions (degrees, 0° = down) ──────────────────────────────────
  public static final double kArmIdleDeg = 0.0;
  public static final double kArmAgitateDownDeg = 0.0;
  public static final double kArmAgitateUpDeg = 30.0;

  // ── Agitate Timing ───────────────────────────────────────────────────────
  public static final double kArmAgitateUpDurationSec = 0.30;
  public static final double kArmAgitateDownDurationSec = 0.30;

  // ── Manual Arm Control (Operator Y/A buttons) ────────────────────────────
  /** Rate at which the arm moves when Y (up) or A (down) is held (°/s). */
  public static final double kArmManualDegPerSec = 30.0;
  /** Upper travel limit for manual arm control (degrees). */
  public static final double kArmMaxDeg = 90.0;

  // ── Arm Pivot Motor Position PID (SparkMax internal) ─────────────────────
  /** Motor rotations per 1 degree of arm travel. *** Update from CAD. */
  public static final double kArmGearRatio = 100.0;
  /** SparkMax internal position PID proportional gain. *** Tune on robot. */
  public static final double kArmKp = 0.1;

  // ── Current Limits ────────────────────────────────────────────────────────
  public static final int kIntakeSmartCurrentLimit = 40;
  public static final int kHopperSmartCurrentLimit = 30;
  public static final int kArmSmartCurrentLimit = 20;
}
