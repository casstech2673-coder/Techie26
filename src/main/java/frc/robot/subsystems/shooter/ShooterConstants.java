// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Constants for the Shooter subsystem.
 *
 * <p>Hardware: 2x Kraken X60 (FOC enabled) at a 1:1 gear ratio. Both motors spin in the same
 * direction to launch the game piece. Hood: position-controlled mechanism on CAN ID 22.
 *
 * <p>Tunable numbers (LoggedTunableNumber) are defined IN Shooter.java so they participate in the
 * AdvantageKit loop. Static constants live here.
 */
public final class ShooterConstants {

  // ── Gear Ratio ────────────────────────────────────────────────────────────
  public static final double kShooterGearRatio = 6.0; // ? need to check again

  // ── Motor CAN IDs ─────────────────────────────────────────────────────────
  public static final int kLeftMotorCanId = 20; // Left flywheel Kraken X60 (leader)
  public static final int kRightMotorCanId = 21; // Right flywheel Kraken X60 (follower)
  public static final int kHoodMotorCanId = 22; // Hood position NEO on SparkMax

  // ── Physical Limits ───────────────────────────────────────────────────────
  public static final double kMaxFlywheelRPM = 6000.0;
  public static final double kStallCurrentAmps = 40.0;

  // ── "Ready to Fire" Tolerance ─────────────────────────────────────────────
  public static final double kReadyToFireRPMTolerance = 50.0; // RPM

  // ── Velocity Recovery Detector ────────────────────────────────────────────
  public static final double kVelocityRecoveryThresholdRPM = 200.0;

  // ── Feed-Forward & PID (flywheel) ─────────────────────────────────────────
  public static final double kFlywheelKs = 0.25;
  public static final double kFlywheelKv = 0.115;
  public static final double kFlywheelKa = 0.01;
  public static final double kFlywheelKp = 0.05;

  // ── Hood Angle Limits ─────────────────────────────────────────────────────
  // 0° = hood fully down (flattest shot, shortest range).
  // 60° = hood fully raised (highest arc, maximum range).
  public static final double kHoodIdleAngleDeg = 0.0; // Default resting position
  public static final double kHoodMinAngleDeg = 0.0;
  public static final double kHoodMaxAngleDeg = 60.0;
  public static final double kHoodToleranceDeg = 1.5; // At-target tolerance

  // ── Hood Motor (NEO on SparkMax, CAN 22) ──────────────────────────────────
  /**
   * Motor-to-hood gear ratio (motor rotations per 1 degree of hood travel). *** Update from CAD.
   */
  public static final double kHoodGearRatio = 50.0;
  /** SparkMax internal position PID proportional gain. *** Tune on robot. */
  public static final double kHoodKp = 0.05;

  // ── Passing Mode ──────────────────────────────────────────────────────────
  // Turret faces the Alliance Wall at global field heading of -180°.
  public static final double kPassRPM = 2500.0; // Flywheel RPM for lob passes
  public static final double kPassHoodAngleDeg = 15.0; // Hood angle for lob passes

  // ── Flywheel Radius (ball-speed ↔ RPM conversion) ─────────────────────────
  public static final double kFlywheelRadiusMeters = 0.051;

  // ── Manual Operator Override ───────────────────────────────────────────────
  /** Fixed flywheel RPM for operator "dumb" shots (no vision). */
  public static final double kManualShootRPM = 3500.0;
  /** Max turret rotation rate when operator stick is at full deflection (°/s). */
  public static final double kManualTurretMaxDegPerSec = 60.0;
  /** Hood angle adjustment rate when operator stick is at full deflection (°/s). */
  public static final double kManualHoodDegPerSec = 15.0;
  /** Default scale multipliers for manual override speeds (tunable via dashboard). */
  public static final double kManualTurretScaleDefault = 1.0;

  public static final double kManualHoodScaleDefault = 1.0;

  // ── Distance → RPM Lookup Table ──────────────────────────────────────────
  // Key   = distance to hub (meters)
  // Value = flywheel RPM setpoint
  // *** PLACEHOLDER SEED VALUES — must be tuned on the actual robot. ***
  public static InterpolatingDoubleTreeMap buildRPMTable() {
    InterpolatingDoubleTreeMap t = new InterpolatingDoubleTreeMap();
    t.put(1.5, 3000.0);
    t.put(2.0, 3300.0);
    t.put(2.5, 3600.0);
    t.put(3.0, 3900.0);
    t.put(3.5, 4200.0);
    t.put(4.0, 4500.0);
    t.put(4.5, 4800.0);
    t.put(5.0, 5100.0);
    t.put(5.5, 5400.0);
    t.put(6.0, 5700.0);
    return t;
  }

  // ── Distance → Hood Angle Lookup Table ───────────────────────────────────
  // Key   = distance to hub (meters)
  // Value = hood angle in degrees (0° = flat, 60° = steep)
  // *** PLACEHOLDER SEED VALUES — must be tuned on the actual robot. ***
  public static InterpolatingDoubleTreeMap buildHoodTable() {
    InterpolatingDoubleTreeMap t = new InterpolatingDoubleTreeMap();
    t.put(1.5, 8.0);
    t.put(2.0, 14.0);
    t.put(2.5, 20.0);
    t.put(3.0, 26.0);
    t.put(3.5, 32.0);
    t.put(4.0, 38.0);
    t.put(4.5, 43.0);
    t.put(5.0, 48.0);
    t.put(5.5, 52.0);
    t.put(6.0, 56.0);
    return t;
  }
}
