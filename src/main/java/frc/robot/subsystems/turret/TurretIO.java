// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware-abstraction interface for the Turret subsystem.
 *
 * <p>Implementations: - {@link TurretIOTalonFX} → Real robot (Talon FX or Kraken driving turret
 * ring gear) - {@link TurretIOSim} → WPILib physics simulation - {@code TurretIO() {}} → Replay
 * mode (no-ops)
 *
 * <p>All position is expressed in DEGREES of turret rotation from home (0°). Positive = clockwise
 * when viewed from above.
 */
public interface TurretIO {

  // ── Logged Inputs ──────────────────────────────────────────────────────────
  @AutoLog
  class TurretIOInputs {

    /** Current turret angle in degrees from home position (0° = forward-facing). */
    public double currentAngleDeg = 0.0;

    /** Current turret angular velocity in degrees per second. */
    public double velocityDegPerSec = 0.0;

    /** Applied output voltage to the turret motor. */
    public double appliedVolts = 0.0;

    /** Stator current draw in amps. */
    public double statorCurrentAmps = 0.0;

    /** Motor temperature in Celsius. */
    public double motorTempCelsius = 0.0;

    /**
     * True when the hardware limit switch at the minimum hard-stop is triggered. Used as the homing
     * reference point — when this trips, zero the encoder.
     */
    public boolean atForwardLimit = false; // "Forward" = kMinAngleDeg direction

    /** True when the hardware limit switch at the maximum hard-stop is triggered. */
    public boolean atReverseLimit = false; // "Reverse" = kMaxAngleDeg direction

    /** True when the motor controller is responding on CAN. */
    public boolean motorConnected = false;

    /** True when the absolute encoder (CANcoder) is responding on CAN. */
    public boolean encoderConnected = false;
  }

  // ── Default (no-op) implementations ───────────────────────────────────────

  /** Refresh inputs with current sensor data. Called once per loop by Turret.periodic(). */
  default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Command the turret to a target angle using closed-loop position control. The implementation
   * enforces hardware soft limits (clamping the target).
   *
   * @param angleDeg Target angle in degrees. Clamped to [kSoftMinAngleDeg, kSoftMaxAngleDeg] inside
   *     the hardware layer as a safety fallback.
   */
  default void setTargetAngle(double angleDeg) {}

  /**
   * Drive the turret at a raw open-loop voltage (used for homing and SysId).
   *
   * @param volts Voltage to apply. Negative = toward kMinAngleDeg.
   */
  default void setVoltage(double volts) {}

  /**
   * Hold the turret at its current position (enable position hold / brake mode). Used when
   * transitioning to HOLD or STOW goals.
   */
  default void holdPosition() {}

  /** Coast the turret motor (neutral output). Only use during fault recovery or manual override. */
  default void stop() {}

  /**
   * Zero the encoder at the current turret position. Called at the end of a successful homing
   * sequence when the limit switch trips.
   */
  default void zeroEncoder() {}
}
