// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public enum Goal {
    IDLE,
    INTAKING,
    KICK,
    JIGGLE,
    EJECT
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private Goal currentGoal = Goal.IDLE;

  private final Timer jiggleTimer = new Timer();
  private boolean jigglePhaseOn = true;

  public Intake(IntakeIO io) {
    this.io = io;
    jiggleTimer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    switch (currentGoal) {
      case IDLE -> {
        io.stopAll();
        jiggleTimer.stop();
        jiggleTimer.reset();
        jigglePhaseOn = true;
      }
      case INTAKING -> {
        io.setIntakeOutput(IntakeConstants.kIntakeSpeed);
        io.setHopperOutput(inputs.hopperBeamBreakTripped ? 0.0 : IntakeConstants.kHopperFeedSpeed);
      }
      case KICK -> {
        io.setKickerOutput(IntakeConstants.kKickerFeedSpeed);
        io.setHopperOutput(IntakeConstants.kHopperFeedSpeed);
      }
      case JIGGLE -> executeJiggle();
      case EJECT -> {
        io.setIntakeOutput(IntakeConstants.kEjectSpeed);
        io.setHopperOutput(-IntakeConstants.kHopperFeedSpeed);
        io.setKickerOutput(IntakeConstants.kKickerEjectSpeed);
      }
    }

    Logger.recordOutput("Intake/Goal", currentGoal.name());
    Logger.recordOutput("Intake/JigglePhaseOn", jigglePhaseOn);
  }

  private void executeJiggle() {
    if (jigglePhaseOn) {
      io.setHopperOutput(IntakeConstants.kHopperSlowSpeed);
      io.setIntakeOutput(0.0);
      io.setKickerOutput(0.0);
      if (jiggleTimer.hasElapsed(IntakeConstants.kJiggleOnDurationSec)) {
        jigglePhaseOn = false;
        jiggleTimer.reset();
      }
    } else {
      io.setHopperOutput(0.0);
      io.setIntakeOutput(0.0);
      io.setKickerOutput(0.0);
      if (jiggleTimer.hasElapsed(IntakeConstants.kJiggleOffDurationSec)) {
        jigglePhaseOn = true;
        jiggleTimer.reset();
      }
    }
  }

  public void setGoal(Goal goal) {
    if (goal != currentGoal && goal != Goal.JIGGLE) {
      jiggleTimer.reset();
      jigglePhaseOn = true;
    }
    this.currentGoal = goal;
  }

  @AutoLogOutput(key = "Intake/HasPieceAtIntake")
  public boolean hasPieceAtIntake() {
    return inputs.intakeBeamBreakTripped;
  }

  @AutoLogOutput(key = "Intake/HasPieceIndexed")
  public boolean hasPieceIndexed() {
    return inputs.hopperBeamBreakTripped;
  }

  @AutoLogOutput(key = "Intake/Goal")
  public Goal getGoal() {
    return currentGoal;
  }
}
