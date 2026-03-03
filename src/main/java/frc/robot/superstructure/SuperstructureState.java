// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

/**
 * All valid operating states for the robot superstructure.
 *
 * <p>The Superstructure is the "Air Traffic Controller" — it owns the state machine and delegates
 * Goal commands to each subsystem.
 *
 * <p>State Transition Diagram:
 *
 * <pre>
 *
 *          ┌─────────────────────────────────────────────┐
 *          │                   IDLE                      │
 *          │  All motors stopped. Turret at STOW (0°).   │
 *          │  Hood returns to 0°.                        │
 *          └─────────────────────────────────────────────┘
 *               │                          │
 *  (Left Trigger — floor intake)  (Y button — pass mode)
 *               ▼                          ▼
 *  ┌─────────────────────┐    ┌───────────────────────────┐
 *  │      INTAKING       │    │          PASSING          │
 *  │  Floor pickup on.   │    │  Turret locked to -180°   │
 *  │  Turret pre-aims.   │    │  field-relative heading.  │
 *  │  Flywheel pre-warm. │    │  Pass RPM + Hood angle.   │
 *  └─────────────────────┘    └───────────────────────────┘
 *          │ (piece indexed)
 *          ▼
 *  ┌─────────────────────────────────────────────────────┐
 *  │                  AIM_AND_SHOOT                      │
 *  │  Unified on-the-move shooting (Right Trigger).      │
 *  │  1. Distance → table → base RPM + hood angle.      │
 *  │  2. Robot velocity subtracted from target vector    │
 *  │     → lead angle + RPM adjustment.                 │
 *  │  3. Stationary & moving handled identically.        │
 *  │  4. Auto-fires when flywheel + turret are ready.   │
 *  └─────────────────────────────────────────────────────┘
 *          │ (isReadyToFire())
 *          ▼
 *  ┌─────────────────────────────────────────────────────┐
 *  │                    SHOOTING                         │
 *  │  Kicker fires. Timer counts down shot cycle.       │
 *  │  Blocked while turret.isWrapping().                │
 *  └─────────────────────────────────────────────────────┘
 *          │ (cycle complete)
 *          ▼
 *    AIM_AND_SHOOT (if RT still held) or IDLE
 *
 *  From ANY state:
 *  ─────────────────────────────────────────────────────
 *  (eject button) → EJECTING → IDLE
 * </pre>
 */
public enum SuperstructureState {

  /** All subsystems at rest. Turret stowed at 0°. Flywheel coasting. Hood at 0°. */
  IDLE,

  /**
   * Actively pulling in a game piece. Intake roller running. Hopper indexing the piece. Turret
   * pre-positions using global pose bearing. Flywheel pre-warms for faster shot readiness.
   * Transition: → AIM_AND_SHOOT once hopperBeamBreak trips.
   */
  INTAKING,

  /**
   * Unified on-the-move hub shooting mode (bound to Right Trigger). Replaces the old trilogy of
   * PRE_SHOOT → STATIONARY_SHOOT / MOVING_SHOOT.
   *
   * <p>Logic: 1. Distance to hub from robot pose + vision → RPM and hood angle via lookup table. 2.
   * Robot chassis velocity subtracted from the field-relative target vector → lead angle and RPM
   * adjustment. 3. On-the-move and stationary shots are identical; when v≈0 the lead angle degrades
   * to a direct aim. 4. Auto-transitions to SHOOTING when isReadyToFire() is true.
   *
   * <p>Intake behaviour is determined by the active {@link Superstructure.IntakeMode}.
   */
  AIM_AND_SHOOT,

  /**
   * Kicker actively firing — game piece in transit to flywheel. Duration controlled by a timer in
   * Superstructure (~0.25 s). Blocked while turret.isWrapping(). Returns to AIM_AND_SHOOT (if
   * button held) or IDLE after cycle.
   */
  SHOOTING,

  /**
   * Alliance Wall alignment / lob-pass mode (bound to Y button). Turret locks to a constant
   * field-relative heading of -180° (facing the Alliance Wall). Flywheel at {@link
   * frc.robot.subsystems.shooter.ShooterConstants#kPassRPM}. Hood at {@link
   * frc.robot.subsystems.shooter.ShooterConstants#kPassHoodAngleDeg}.
   *
   * <p>Intake behaviour is determined by the active {@link Superstructure.IntakeMode}.
   */
  PASSING,

  /**
   * Forced reversal — clears jams or ejects bad game pieces. All transport motors run in reverse.
   * Turret holds position (no tracking during eject).
   */
  EJECTING,

  /**
   * Operator manual override. Turret driven incrementally by operator right stick X. Hood angle
   * adjusted incrementally by operator left stick Y. Flywheel spins at kManualShootRPM when
   * operator RT is held. Intake still routed through IntakeMode. Interrupts AimAndShoot / Pass.
   */
  MANUAL_CONTROL
}
