// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;

/**
 * ShootCommand — driver-initiated shooting sequence.
 *
 * <p>This command uses the "Request" pattern: - It tells {@link Superstructure} what STATE to be
 * in. - The Superstructure's periodic() handles all the logic. - This command only manages the
 * lifecycle (start / end conditions).
 *
 * <p>This is intentionally thin. Commands should NOT contain shooting logic; that lives in
 * Superstructure.periodic(). Commands are just triggers.
 *
 * <p>Usage in RobotContainer:
 *
 * <pre>
 *   // Hold right bumper to shoot:
 *   controller.rightBumper()
 *       .whileTrue(new ShootCommand(superstructure));
 * </pre>
 *
 * <p>State lifecycle: - initialize(): Request PRE_SHOOT (begin spin-up) - execute(): Superstructure
 * auto-transitions through PRE_SHOOT → STATIONARY/MOVING → SHOOTING - end(): Request IDLE (spin
 * down, turret stow) - isFinished(): Never finishes early (button-held pattern)
 */
public class ShootCommand extends Command {

  private final Superstructure superstructure;

  /**
   * @param superstructure The robot's Superstructure (air traffic controller). NOTE: This command
   *     does NOT require individual subsystems — Superstructure owns them; requiring Superstructure
   *     is sufficient if Superstructure is declared as a requirement.
   */
  public ShootCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    // PSEUDOCODE:
    // addRequirements(superstructure);
    // If Superstructure extends SubsystemBase and is registered, add it as a requirement.
    // This prevents other commands from using the superstructure simultaneously.
  }

  @Override
  public void initialize() {
    // PSEUDOCODE:
    //
    // Request spin-up. Superstructure.periodic() will:
    //   1. Spin up the flywheel.
    //   2. Start tracking with vision.
    //   3. Enable hopper jiggle.
    //   4. Auto-transition to STATIONARY or MOVING shot when ready.
    //
    // superstructure.setGoal(SuperstructureState.PRE_SHOOT);
    superstructure.setGoal(SuperstructureState.PRE_SHOOT);
  }

  @Override
  public void execute() {
    // PSEUDOCODE — nothing needed here:
    //
    // Superstructure.periodic() runs automatically via the CommandScheduler.
    // It handles ALL the state transitions internally (PRE_SHOOT → SHOOTING).
    // We just hold the goal — the superstructure will fire when ready.
    //
    // Optional: override the shoot trigger based on driver input
    // (e.g., driver must confirm the shot with a second button):
    //   if (driverConfirmsShot && superstructure.isSystemReady()) {
    //       // Superstructure already auto-fires when isReadyToFire() is true.
    //       // No action needed here unless you want a MANUAL fire override.
    //   }
  }

  @Override
  public void end(boolean interrupted) {
    // PSEUDOCODE:
    //
    // Whether the command ends normally or is interrupted (button released),
    // return to IDLE state.
    //   - Flywheels coast to stop.
    //   - Turret stows.
    //   - Hopper jiggle stops.
    //
    // superstructure.setGoal(SuperstructureState.IDLE);
    superstructure.setGoal(SuperstructureState.IDLE);
  }

  @Override
  public boolean isFinished() {
    // PSEUDOCODE:
    //
    // This is a "hold-button" command — it runs as long as the button is held.
    // Return false to never self-terminate.
    //
    // ALTERNATIVE: Return true after a SHOOTING state completes (one-shot):
    //   return superstructure.getActiveState() == SuperstructureState.IDLE
    //          && previousState == SuperstructureState.SHOOTING;
    //
    // For now: simple hold pattern.
    return false;
  }
}
