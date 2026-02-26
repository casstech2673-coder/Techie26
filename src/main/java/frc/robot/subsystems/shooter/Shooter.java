// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {

  public enum Goal {
    IDLE(0.0),
    SHOOTING(4500.0),
    SHOOTING_MOVING(-1.0),
    FEEDING(500.0),
    EJECTING(-1500.0);

    public final double defaultRPM;

    Goal(double defaultRPM) {
      this.defaultRPM = defaultRPM;
    }
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private Goal currentGoal = Goal.IDLE;
  private double customRPM = 0.0;
  private boolean turretAligned = false;

  private static final LoggedNetworkNumber tunableKp =
      new LoggedNetworkNumber("Shooter/kP", ShooterConstants.kFlywheelKp);
  private static final LoggedNetworkNumber tunableKs =
      new LoggedNetworkNumber("Shooter/kS", ShooterConstants.kFlywheelKs);
  private static final LoggedNetworkNumber tunableKv =
      new LoggedNetworkNumber("Shooter/kV", ShooterConstants.kFlywheelKv);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    switch (currentGoal) {
      case IDLE -> io.stop();
      case SHOOTING -> io.setFlywheelVelocity(Goal.SHOOTING.defaultRPM);
      case SHOOTING_MOVING -> io.setFlywheelVelocity(
          customRPM > 0.0 ? customRPM : Goal.SHOOTING.defaultRPM);
      case FEEDING -> io.setFlywheelVelocity(Goal.FEEDING.defaultRPM);
      case EJECTING -> io.setFlywheelVelocity(Goal.EJECTING.defaultRPM);
    }

    Logger.recordOutput("Shooter/Goal", currentGoal.name());
    Logger.recordOutput("Shooter/IsReadyToFire", isReadyToFire());
    Logger.recordOutput("Shooter/VelocityErrorRPM", getVelocityErrorRPM());
    Logger.recordOutput("Shooter/AverageRPM", getAverageRPM());
  }

  public void setGoal(Goal goal) {
    this.currentGoal = goal;
  }

  public void setCustomRPM(double rpm) {
    this.customRPM = rpm;
  }

  public void setTurretAligned(boolean aligned) {
    this.turretAligned = aligned;
  }

  @AutoLogOutput(key = "Shooter/ReadyToFire")
  public boolean isReadyToFire() {
    if (currentGoal == Goal.IDLE || currentGoal == Goal.EJECTING) return false;
    double target = getTargetRPM();
    double errTop = Math.abs(inputs.flywheelVelocityRPM[0] - target);
    double errBot = Math.abs(inputs.flywheelVelocityRPM[1] - target);
    return errTop <= ShooterConstants.kReadyToFireRPMTolerance
        && errBot <= ShooterConstants.kReadyToFireRPMTolerance
        && turretAligned;
  }

  @AutoLogOutput(key = "Shooter/AverageVelocityErrorRPM")
  public double getVelocityErrorRPM() {
    double target = getTargetRPM();
    return (Math.abs(inputs.flywheelVelocityRPM[0] - target)
            + Math.abs(inputs.flywheelVelocityRPM[1] - target))
        / 2.0;
  }

  @AutoLogOutput(key = "Shooter/AverageRPM")
  public double getAverageRPM() {
    return (inputs.flywheelVelocityRPM[0] + inputs.flywheelVelocityRPM[1]) / 2.0;
  }

  @AutoLogOutput(key = "Shooter/Goal")
  public Goal getGoal() {
    return currentGoal;
  }

  private double getTargetRPM() {
    if (currentGoal == Goal.SHOOTING_MOVING && customRPM > 0.0) return customRPM;
    return currentGoal.defaultRPM;
  }
}
