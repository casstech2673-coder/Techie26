// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;

/**
 * ShootCommand — compatibility alias for {@link AimAndShootCommand}.
 *
 * <p>Requests {@link SuperstructureState#AIM_AND_SHOOT} for the duration the button is held. Prefer
 * {@link AimAndShootCommand} for new bindings.
 *
 * <p>Usage:
 *
 * <pre>
 *   controller.rightTrigger(0.5).whileTrue(new ShootCommand(superstructure));
 * </pre>
 */
public class ShootCommand extends Command {

  private final Superstructure superstructure;

  public ShootCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.setGoal(SuperstructureState.AIM_AND_SHOOT);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    superstructure.setGoal(SuperstructureState.IDLE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
