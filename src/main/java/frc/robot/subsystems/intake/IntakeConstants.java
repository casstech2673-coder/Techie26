// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/**
 * Constants for the Intake / Kicker / Hopper subsystem.
 *
 * <p>Hardware: - Intake roller: NEO 1.1 on SparkMax (ground-facing, pulls game piece in) - Hopper
 * belts: NEO 1.1 on SparkFlex (carries game piece from intake → shooter) - Kicker: NEO 1.1 on
 * SparkMax (feeds game piece into the flywheel gap)
 *
 * <p>Sensors: - Intake beam break: detects when a game piece enters the intake. - Hopper beam
 * break: detects when a game piece is at the handoff to the shooter.
 *
 * <p>The "Jiggle" pattern: When the robot is stationary and has a piece but is NOT yet commanded to
 * shoot, the hopper runs a periodic toggle to prevent the game piece from resting on a seam and
 * causing a feed failure.
 */
public final class IntakeConstants {

  // ── Motor CAN IDs ────────────────────────────────────────────────────────
  public static final int kIntakeMotorCanId = 40;
  public static final int kHopperMotorCanId = 41;
  public static final int kKickerMotorCanId = 42;

  // ── Duty Cycle Setpoints (percent output, -1.0 to +1.0) ──────────────────
  public static final double kIntakeSpeed = 0.85; // Full-send intake
  public static final double kEjectSpeed = -0.85; // Reverse to eject
  public static final double kHopperFeedSpeed = 0.6; // Normal hopper index speed
  public static final double kHopperSlowSpeed = 0.2; // Slow advance during jiggle ON phase
  public static final double kKickerFeedSpeed = 0.9; // Feed the game piece into shooter
  public static final double kKickerEjectSpeed = -0.9; // Reverse kicker for ejection

  // ── Jiggle Timer Parameters ───────────────────────────────────────────────
  // "Jiggle" prevents the game piece from sitting stationary on a belt seam.
  // The hopper alternates between slow-forward and full-stop.
  //
  // Only active when: SuperstructureState == STATIONARY_SHOOT && piece is indexed.
  public static final double kJiggleOnDurationSec = 0.15; // How long to run forward
  public static final double kJiggleOffDurationSec = 0.35; // How long to pause
  // Full jiggle cycle period = kJiggleOnDurationSec + kJiggleOffDurationSec = 0.5 s

  // ── Current Limits (NEO on SparkMax) ─────────────────────────────────────
  public static final int kIntakeSmartCurrentLimit = 40; // Amps
  public static final int kHopperSmartCurrentLimit = 30; // Amps
  public static final int kKickerSmartCurrentLimit = 35; // Amps

  // ── DIO Port Assignments (beam break sensors) ─────────────────────────────
  public static final int kIntakeBeamBreakPort = 0; // DIO channel 0
  public static final int kHopperBeamBreakPort = 1; // DIO channel 1
}
