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
 *                     ┌──────────────────────────────────────────┐
 *                     │                  IDLE                    │
 *                     │  (All motors stopped, turret at STOW)    │
 *                     └──────────────────────────────────────────┘
 *                          │ (driver presses intake button)
 *                          ▼
 *                     ┌──────────────────────────────────────────┐
 *                     │               INTAKING                   │
 *                     │  Intake runs, hopper indexes piece.       │
 *                     │  Turret pre-positions to field estimate.  │
 *                     └──────────────────────────────────────────┘
 *                          │ (hopperBeamBreak trips = piece indexed)
 *                          ▼
 *                     ┌──────────────────────────────────────────┐
 *                     │             PRE_SHOOT                    │
 *                     │  Flywheel spinning up. Turret tracking.  │
 *                     │  Hopper in JIGGLE mode.                  │
 *                     └──────────────────────────────────────────┘
 *                 ┌────────────┘              └──────────────────┐
 *    (robot stopped)                              (robot moving)
 *                 ▼                                              ▼
 *  ┌──────────────────────────────┐     ┌──────────────────────────────────┐
 *  │       STATIONARY_SHOOT       │     │          MOVING_SHOOT            │
 *  │  Final aim lock.  Jiggle on. │     │  Vector math applied to RPM/aim. │
 *  │  Kicker armed once ready.    │     │  Shoot only when lead settled.   │
 *  └──────────────────────────────┘     └──────────────────────────────────┘
 *                 │                                              │
 *      (isReadyToFire())                             (isReadyToFire())
 *                 └─────────────────┬────────────────────────────┘
 *                                   ▼
 *                     ┌──────────────────────────────────────────┐
 *                     │              SHOOTING                    │
 *                     │  Kicker fires. Timer counts shot cycle.  │
 *                     │  Blocked if turret.isWrapping() == true. │
 *                     └──────────────────────────────────────────┘
 *                          │ (shot cycle complete)
 *                          ▼
 *                     IDLE or INTAKING (based on driver input)
 *
 *  From ANY state:
 *  ─────────────────────────────────────────
 *  (driver holds eject button) → EJECTING → IDLE
 * </pre>
 */
public enum SuperstructureState {

  /**
   * All subsystems at rest. Safe for driving without game piece concerns. Turret stowed at 0°
   * (forward-facing). Flywheel coasting.
   */
  IDLE,

  /**
   * Actively pulling in a game piece. Intake roller running, hopper indexing. Turret pre-positions
   * using global pose bearing (prepares for quick shot). Flywheel begins slow spin-up (pre-warm for
   * faster ready time).
   */
  INTAKING,

  /**
   * Game piece is indexed. Robot is preparing to shoot. Flywheel spins up to target RPM. Turret
   * actively tracking (FIELD_TRACKING or TARGET_LOCK). Hopper in JIGGLE mode. Transition: →
   * STATIONARY_SHOOT or MOVING_SHOOT based on chassis speed.
   */
  PRE_SHOOT,

  /**
   * Robot is stationary (chassis speed below threshold). Tightest aim accuracy expected. Jiggle
   * active. Kicker armed when isReadyToFire().
   */
  STATIONARY_SHOOT,

  /**
   * Robot is moving. Shooting on the move. Superstructure applies vector math to compensate for
   * robot velocity. Adjusted RPM and turret lead angle sent to subsystems.
   */
  MOVING_SHOOT,

  /**
   * Kicker is actively firing — game piece in transit to flywheel. Blocked by: turret.isWrapping()
   * == true. Duration: ~0.15-0.3 seconds (controlled by a timer in Superstructure). After shot
   * cycle: returns to IDLE or INTAKING.
   */
  SHOOTING,

  /**
   * Forced reversal — clears jams or spits out a bad game piece. All transport motors run in
   * reverse. Turret holds position (doesn't track during eject).
   */
  EJECTING
}
