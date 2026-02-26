// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/**
 * Constants for the Shooter subsystem.
 *
 * <p>Hardware: 2x Kraken X60 (FOC enabled) at a 1:1 gear ratio. Both motors spin in the same
 * direction to launch the game piece.
 *
 * <p>Tunable numbers (LoggedTunableNumber) are defined IN Shooter.java so they participate in the
 * AdvantageKit loop. Static constants live here.
 */
public final class ShooterConstants {

  // ── Gear Ratio ────────────────────────────────────────────────────────────
  // Made a variable so it can be changed without touching every calculation.
  // 1.0 = motor shaft directly drives the flywheel (1:1).
  public static final double kShooterGearRatio = 1.0;

  // ── Motor CAN IDs ─────────────────────────────────────────────────────────
  public static final int kTopMotorCanId = 20; // Top flywheel Kraken X60
  public static final int kBottomMotorCanId = 21; // Bottom flywheel Kraken X60

  // ── Physical Limits ───────────────────────────────────────────────────────
  public static final double kMaxFlywheelRPM = 6000.0; // Kraken X60 free speed ≈ 6000 RPM
  public static final double kStallCurrentAmps = 40.0; // Per-motor stall-current limit

  // ── "Ready to Fire" Tolerance ─────────────────────────────────────────────
  // The shooter is considered spun-up when BOTH motors are within this
  // window of their setpoint.  ±50 RPM gives fast-cycle reliability.
  public static final double kReadyToFireRPMTolerance = 50.0; // RPM

  // ── Velocity Recovery Detector ────────────────────────────────────────────
  // After a ball passes through the shooter the RPM dips.  If the RPM error
  // exceeds this threshold we know a ball has just been kicked and we start
  // the recovery clock.
  public static final double kVelocityRecoveryThresholdRPM = 200.0;

  // ── Feed-Forward & PID (starting values — override via LoggedTunableNumber) ─
  // These are the initial values loaded into the on-robot tuning numbers.
  // Adjust them in Shooter.java's LoggedTunableNumber declarations.
  public static final double kFlywheelKs = 0.25; // Static friction (volts)
  public static final double kFlywheelKv = 0.115; // Velocity gain   (V / (rot/s))
  public static final double kFlywheelKa = 0.01; // Acceleration gain
  public static final double kFlywheelKp = 0.05; // Proportional gain
}
