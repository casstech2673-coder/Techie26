// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;

/**
 * AimAndShootCommand — unified hub-targeting shot.
 *
 * <p>Merges the old stationary + moving shot into a single command. While this command is active,
 * Superstructure computes:
 *
 * <ul>
 *   <li>Distance to hub → flywheel RPM + hood angle via interpolating lookup tables.
 *   <li>Robot chassis velocity → turret lead angle + RPM compensation (on-the-move logic).
 *   <li>Auto-fires when flywheel is at speed AND turret is aligned.
 * </ul>
 *
 * <p>Bound to: <b>Right Trigger (&gt; 0.5 threshold)</b>.
 *
 * <p>The intake mode (floor pickup vs. agitate vs. idle) is controlled independently via {@link
 * Superstructure#setIntakeMode(Superstructure.IntakeMode)} — see RobotContainer for the four
 * composite bindings.
 *
 * <p>Usage:
 *
 * <pre>
 *   controller.rightTrigger(0.5).whileTrue(new AimAndShootCommand(superstructure));
 * </pre>
 */
public class AimAndShootCommand extends Command {

  private final Superstructure superstructure;

  public AimAndShootCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.setGoal(SuperstructureState.AIM_AND_SHOOT);
  }

  @Override
  public void execute() {
    // Superstructure.periodic() drives all logic automatically.
    // Nothing needed here — the goal is already set.
  }

  @Override
  public void end(boolean interrupted) {
    // Return to idle when button is released or command is interrupted.
    superstructure.setGoal(SuperstructureState.IDLE);
  }

  @Override
  public boolean isFinished() {
    // Hold-button pattern: runs as long as Right Trigger is held.
    return false;
  }
}
