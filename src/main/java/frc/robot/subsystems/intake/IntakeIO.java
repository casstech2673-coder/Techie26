// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware-abstraction interface for the Intake / Kicker / Hopper subsystem.
 *
 * <p>All three actuators (intake roller, hopper, kicker) are grouped into one IO interface because
 * they form a single "game piece transport" system and rarely operate independently of each other.
 *
 * <p>Implementations: - {@link IntakeIOSparkMax} → Real robot (NEO on SparkMax/Flex) - {@link
 * IntakeIOSim} → WPILib physics simulation - {@code IntakeIO() {}} → Replay mode (no-ops)
 */
public interface IntakeIO {

  // ── Logged Inputs ──────────────────────────────────────────────────────────
  @AutoLog
  class IntakeIOInputs {

    /** Intake roller velocity in RPM. Positive = intaking direction. */
    public double intakeVelocityRPM = 0.0;

    /** Hopper belt velocity in RPM. */
    public double hopperVelocityRPM = 0.0;

    /** Kicker wheel velocity in RPM. */
    public double kickerVelocityRPM = 0.0;

    /** Applied output for intake motor (-1.0 to +1.0). */
    public double intakeAppliedOutput = 0.0;

    /** Applied output for hopper motor (-1.0 to +1.0). */
    public double hopperAppliedOutput = 0.0;

    /** Applied output for kicker motor (-1.0 to +1.0). */
    public double kickerAppliedOutput = 0.0;

    /** Stator current in amps for the intake motor. Useful for detecting jams. */
    public double intakeCurrentAmps = 0.0;

    /** Stator current in amps for the hopper motor. */
    public double hopperCurrentAmps = 0.0;

    /** Stator current in amps for the kicker motor. */
    public double kickerCurrentAmps = 0.0;

    /**
     * True when the intake beam break is interrupted — a game piece has entered the intake zone.
     */
    public boolean intakeBeamBreakTripped = false;

    /**
     * True when the hopper beam break is interrupted — a game piece is staged and ready at the
     * kicker handoff point (fully indexed).
     */
    public boolean hopperBeamBreakTripped = false;

    /** True when all three SparkMax/Flex controllers are responding. */
    public boolean[] motorConnected = new boolean[] {false, false, false};
  }

  // ── Default (no-op) implementations ───────────────────────────────────────

  /** Refresh inputs. Called once per loop by Intake.periodic(). */
  default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Set the intake roller speed.
   *
   * @param output Percent output (-1.0 to +1.0). Positive = intake direction.
   */
  default void setIntakeOutput(double output) {}

  /**
   * Set the hopper belt speed.
   *
   * @param output Percent output (-1.0 to +1.0).
   */
  default void setHopperOutput(double output) {}

  /**
   * Set the kicker wheel speed.
   *
   * @param output Percent output (-1.0 to +1.0). Positive = feed-into-shooter.
   */
  default void setKickerOutput(double output) {}

  /** Stop all three motors simultaneously. Called during IDLE goal. */
  default void stopAll() {}
}
