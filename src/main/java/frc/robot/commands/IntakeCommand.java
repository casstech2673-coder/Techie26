// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;

/** Floor pickup — holds INTAKING while button is pressed, returns to IDLE on release. */
public class IntakeCommand extends Command {

  private final Superstructure superstructure;

  public IntakeCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.setGoal(SuperstructureState.INTAKING);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setGoal(SuperstructureState.IDLE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
