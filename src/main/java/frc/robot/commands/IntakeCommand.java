// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;

/**
 * IntakeCommand — driver-initiated game piece intake sequence.
 *
 * <p>Like {@link ShootCommand}, this delegates ALL logic to {@link Superstructure}. The command
 * sets the goal; the Superstructure executes and auto-transitions.
 *
 * <p>Usage in RobotContainer:
 *
 * <pre>
 *   // Hold left bumper to intake:
 *   controller.leftBumper()
 *       .whileTrue(new IntakeCommand(superstructure));
 * </pre>
 *
 * <p>Auto-end condition: The command finishes itself when the Superstructure has detected that the
 * game piece is fully indexed (hopper beam break tripped) and auto-transitioned from INTAKING →
 * PRE_SHOOT. At that point, the intake command is "done" and the driver can release the button
 * naturally.
 */
public class IntakeCommand extends Command {

  private final Superstructure superstructure;

  public IntakeCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    // PSEUDOCODE: addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    // PSEUDOCODE:
    // Request INTAKING state. Superstructure.periodic() will:
    //   1. Run intake roller.
    //   2. Run hopper to index the piece.
    //   3. Pre-position the turret using global pose bearing.
    //   4. Slowly pre-warm the flywheel for faster shot readiness.
    //   5. Auto-transition to PRE_SHOOT once hopperBeamBreak trips.
    //
    // superstructure.setGoal(SuperstructureState.INTAKING);
    superstructure.setGoal(SuperstructureState.INTAKING);
  }

  @Override
  public void execute() {
    // Nothing needed — Superstructure handles everything.
  }

  @Override
  public void end(boolean interrupted) {
    // PSEUDOCODE:
    // If the driver releases the button before a piece is indexed,
    // return to IDLE (stop intake motors).
    // If the button is released AFTER indexing (isFinished() returned true),
    // the superstructure is already in PRE_SHOOT — do NOT override to IDLE.
    //
    // if (interrupted) {
    //     superstructure.setGoal(SuperstructureState.IDLE);
    // }
    // (When isFinished() triggers, end() is also called with interrupted=false.
    //  In that case the superstructure has already moved to PRE_SHOOT — leave it.)
    if (interrupted) {
      superstructure.setGoal(SuperstructureState.IDLE);
    }
  }

  @Override
  public boolean isFinished() {
    // PSEUDOCODE:
    // The intake sequence is complete once the piece is indexed and
    // Superstructure has auto-transitioned to PRE_SHOOT (or further).
    //
    // return superstructure.getActiveState() == SuperstructureState.PRE_SHOOT
    //     || superstructure.getActiveState() == SuperstructureState.STATIONARY_SHOOT
    //     || superstructure.getActiveState() == SuperstructureState.MOVING_SHOOT;
    return superstructure.getActiveState() == SuperstructureState.PRE_SHOOT;
  }
}
