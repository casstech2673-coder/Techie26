// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;

/**
 * PassCommand — Alliance Wall alignment lob-pass mode.
 *
 * <p>While active, Superstructure:
 *
 * <ul>
 *   <li>Locks the turret to a constant <b>field-relative heading of -180°</b> (facing the Alliance
 *       Wall), regardless of robot orientation.
 *   <li>Spins the flywheel to {@link frc.robot.subsystems.shooter.ShooterConstants#kPassRPM}.
 *   <li>Raises the hood to {@link frc.robot.subsystems.shooter.ShooterConstants#kPassHoodAngleDeg}.
 *   <li>Auto-fires when turret is aligned and flywheel is at speed.
 * </ul>
 *
 * <p>Bound to: <b>Y button</b> (Alliance Wall Face).
 *
 * <p>The intake mode (floor pickup vs. agitate vs. idle) is controlled independently via {@link
 * Superstructure#setIntakeMode(Superstructure.IntakeMode)} — see RobotContainer.
 *
 * <p>Usage:
 *
 * <pre>
 *   controller.y().whileTrue(new PassCommand(superstructure));
 * </pre>
 */
public class PassCommand extends Command {

  private final Superstructure superstructure;

  public PassCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.setGoal(SuperstructureState.PASSING);
  }

  @Override
  public void execute() {
    // Superstructure.periodic() holds the -180° turret heading automatically.
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setGoal(SuperstructureState.IDLE);
  }

  @Override
  public boolean isFinished() {
    // Hold-button pattern: runs as long as Y is held.
    return false;
  }
}
