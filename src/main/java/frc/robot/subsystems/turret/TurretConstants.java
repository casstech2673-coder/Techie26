// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

/**
 * Constants for the Turret subsystem.
 *
 * <p>The turret is a hard-stop limited mechanism — it physically CANNOT rotate past {@link
 * #kMinAngleDeg} or {@link #kMaxAngleDeg}. All angles are measured in degrees from the turret's
 * forward-facing "home" position (0°).
 *
 * <p>Soft limits provide a software-enforced buffer INSIDE the hard stops to prevent the mechanism
 * from slamming into them at speed.
 *
 * <p>Wrapping logic handles the case where the optimal path to a target would cross a hard stop —
 * instead the turret rotates the other direction.
 */
public final class TurretConstants {

  // ── Hard Stop Limits ────────────────────────────────────────────────────────
  // These represent the physical endpoints of turret travel.
  // DO NOT command the turret beyond these angles.
  public static final double kMinAngleDeg = -180.0; // Full left (CCW)
  public static final double kMaxAngleDeg = 180.0; // Full right (CW)

  // ── Soft Limit Buffer ───────────────────────────────────────────────────────
  // The turret will NOT be commanded within this margin of the hard stops.
  // Example: With kMaxAngleDeg=180 and kSoftLimitBufferDeg=10,
  // the maximum commanded angle is 170°.
  public static final double kSoftLimitBufferDeg = 10.0;

  // Derived soft limit boundaries (computed from above constants):
  public static final double kSoftMinAngleDeg = kMinAngleDeg + kSoftLimitBufferDeg; // -170°
  public static final double kSoftMaxAngleDeg = kMaxAngleDeg - kSoftLimitBufferDeg; // +170°

  // ── Wrapping Zone Threshold ─────────────────────────────────────────────────
  // If the target angle is within this many degrees of a soft limit,
  // the wrapping decision algorithm kicks in to evaluate whether to wrap
  // to the equivalent angle on the other side instead of approaching the limit.
  // Example: Target = 175°, threshold = 15° → trigger wrap check at 170 - 15 = 155°.
  public static final double kWrappingZoneDeg = 15.0;

  // ── "At Target" Alignment Tolerance ────────────────────────────────────────
  // Shooter.isReadyToFire() uses this: turret must be within ±kAlignedToleranceDeg.
  public static final double kAlignedToleranceDeg = 2.0;

  // ── Homing ─────────────────────────────────────────────────────────────────
  // Homing moves the turret toward the hard stop until the limit switch trips.
  // After homing, the encoder is zeroed at the limit switch position.
  public static final double kHomingVoltage = -2.0; // Volts (toward min limit)
  public static final double kHomingMaxSpeedDegS = 30.0; // °/s ceiling during homing

  // ── Motor CAN ID ───────────────────────────────────────────────────────────
  public static final int kTurretMotorCanId = 30;

  // ── Encoder ────────────────────────────────────────────────────────────────
  // The turret may use an absolute encoder (e.g., CANcoder) for reliable
  // position-on-boot without requiring a homing sequence every time.
  public static final int kTurretEncoderCanId = 31;

  // Ticks or rotations per degree of turret rotation (depends on gear ratio).
  // Example: If turret gearbox is 100:1 and TalonFX reads motor rotations:
  //   kTurretGearRatio = 100.0  → 100 motor rotations = 1 turret rotation = 360°
  public static final double kTurretGearRatio = 60.0; // Motor rotations per turret rotation

  // PID / Profile starting values (overridable via LoggedTunableNumber in Turret.java)
  public static final double kTurretKp = 8.0; // P gain (output per degree of error)
  public static final double kTurretKd = 0.2; // D gain (dampens oscillation)
  public static final double kTurretMaxVelocityDegS = 300.0; // Motion profile max velocity
  public static final double kTurretMaxAccelDegS2 = 600.0; // Motion profile max acceleration
}
