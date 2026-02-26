// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware-abstraction interface for the Shooter subsystem.
 *
 * <p>This follows the AdvantageKit IO layer pattern: 1. All sensor data flows OUT through {@link
 * ShooterIOInputs} (logged every loop). 2. All actuator commands flow IN through the setter methods
 * below.
 *
 * <p>Implementations: - {@link ShooterIOKrakenFOC} → Real robot (Kraken X60 via TalonFX) - {@link
 * ShooterIOSim} → WPILib physics simulation - {@code ShooterIO() {}} → Replay mode (no-ops, inputs
 * come from log)
 */
public interface ShooterIO {

  // ── Logged Inputs ──────────────────────────────────────────────────────────
  // @AutoLog generates ShooterIOInputsAutoLogged, which AdvantageKit
  // automatically logs every loop iteration via Logger.processInputs().
  @AutoLog
  class ShooterIOInputs {

    // Index 0 = top motor, index 1 = bottom motor.
    // Arrays let us log both motors with a single field.

    /** Flywheel velocity for each motor, in Rotations Per Minute (RPM). */
    public double[] flywheelVelocityRPM = new double[] {0.0, 0.0};

    /** Voltage currently applied to each motor (for diagnostics and replay). */
    public double[] flywheelAppliedVolts = new double[] {0.0, 0.0};

    /** Stator (motor-side) current draw in amps — useful for jam detection. */
    public double[] statorCurrentAmps = new double[] {0.0, 0.0};

    /** Supply (battery-side) current draw in amps — useful for brownout detection. */
    public double[] supplyCurrentAmps = new double[] {0.0, 0.0};

    /** Motor temperature in Celsius — used for thermal de-rating warnings. */
    public double[] motorTempCelsius = new double[] {0.0, 0.0};

    /** True when the motor controller is responding on CAN (health check). */
    public boolean[] motorConnected = new boolean[] {false, false};

    /**
     * True when the flywheel velocity has recovered after a ball passed through. Set high once RPM
     * returns to within {@code kReadyToFireRPMTolerance} of setpoint following a {@code
     * kVelocityRecoveryThresholdRPM} dip.
     */
    public boolean velocityRecovered = false;
  }

  // ── Default (no-op) implementations ───────────────────────────────────────
  // All methods are default so the anonymous "replay" implementation
  // ({@code new ShooterIO() {}}) compiles without any method bodies.

  /**
   * Refresh the {@link ShooterIOInputs} struct with the latest sensor data. Called ONCE per loop by
   * {@link Shooter#periodic()}.
   */
  default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Command both flywheels to a target velocity using closed-loop control.
   *
   * @param rpm Target rotational speed in RPM. Positive = launch direction. The implementation
   *     converts RPM → rotations/sec before sending to TalonFX's VelocityVoltage or
   *     VelocityTorqueCurrentFOC request.
   */
  default void setFlywheelVelocity(double rpm) {}

  /**
   * Drive both flywheels at a raw open-loop voltage. Used during SysId characterization routines.
   *
   * @param volts Voltage to apply (-12 to +12).
   */
  default void setFlywheelVoltage(double volts) {}

  /**
   * Spin each motor independently. Allows differential top/bottom speed for backspin or topspin
   * shot shaping.
   *
   * @param topRPM Target RPM for the top flywheel.
   * @param bottomRPM Target RPM for the bottom flywheel.
   */
  default void setFlywheelVelocityIndependent(double topRPM, double bottomRPM) {}

  /**
   * Coast both flywheels to a stop (neutral output). Used when transitioning to {@link
   * Shooter.Goal#IDLE}.
   */
  default void stop() {}
}
