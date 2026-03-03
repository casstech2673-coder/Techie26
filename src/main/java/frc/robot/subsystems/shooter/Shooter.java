// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {

  /**
   * High-level flywheel goals.
   *
   * <p>PASSING and AIM_AND_SHOOT use dynamically-set RPM from Superstructure. The defaultRPM field
   * is a safe fallback only.
   */
  public enum Goal {
    IDLE(0.0),
    AIM_AND_SHOOT(-1.0), // RPM set dynamically via setCustomRPM()
    PASSING(ShooterConstants.kPassRPM),
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
  private double targetHoodAngleDeg = ShooterConstants.kHoodIdleAngleDeg;
  private boolean turretAligned = false;

  private static final LoggedNetworkNumber tunableKp =
      new LoggedNetworkNumber("Shooter/kP", ShooterConstants.kFlywheelKp);
  private static final LoggedNetworkNumber tunableKs =
      new LoggedNetworkNumber("Shooter/kS", ShooterConstants.kFlywheelKs);
  private static final LoggedNetworkNumber tunableKv =
      new LoggedNetworkNumber("Shooter/kV", ShooterConstants.kFlywheelKv);
  private static final LoggedNetworkNumber tunableKa =
      new LoggedNetworkNumber("Shooter/kA", ShooterConstants.kFlywheelKa);
  public static final LoggedNetworkNumber tunableManualTurretScale =
      new LoggedNetworkNumber(
          "Shooter/ManualTurretScale", ShooterConstants.kManualTurretScaleDefault);
  public static final LoggedNetworkNumber tunableManualHoodScale =
      new LoggedNetworkNumber("Shooter/ManualHoodScale", ShooterConstants.kManualHoodScaleDefault);

  // Track last-applied PID values to avoid re-applying unchanged gains every loop
  private double lastKp = ShooterConstants.kFlywheelKp;
  private double lastKs = ShooterConstants.kFlywheelKs;
  private double lastKv = ShooterConstants.kFlywheelKv;
  private double lastKa = ShooterConstants.kFlywheelKa;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Apply PID gains to hardware only when tunable values change
    double kp = tunableKp.get();
    double ks = tunableKs.get();
    double kv = tunableKv.get();
    double ka = tunableKa.get();
    if (kp != lastKp || ks != lastKs || kv != lastKv || ka != lastKa) {
      io.setPIDGains(ks, kv, ka, kp);
      lastKp = kp;
      lastKs = ks;
      lastKv = kv;
      lastKa = ka;
    }

    switch (currentGoal) {
      case IDLE -> {
        io.stop();
        io.setHoodAngle(ShooterConstants.kHoodIdleAngleDeg); // Hood returns to 0° when idle
      }
      case AIM_AND_SHOOT -> {
        double rpm = customRPM > 0.0 ? customRPM : 4500.0; // fallback if table not ready
        io.setFlywheelVelocity(rpm);
        io.setHoodAngle(
            MathUtil.clamp(
                targetHoodAngleDeg,
                ShooterConstants.kHoodMinAngleDeg,
                ShooterConstants.kHoodMaxAngleDeg));
      }
      case PASSING -> {
        io.setFlywheelVelocity(ShooterConstants.kPassRPM);
        io.setHoodAngle(ShooterConstants.kPassHoodAngleDeg);
      }
      case FEEDING -> {
        io.setFlywheelVelocity(Goal.FEEDING.defaultRPM);
        io.setHoodAngle(ShooterConstants.kHoodIdleAngleDeg);
      }
      case EJECTING -> {
        io.setFlywheelVelocity(Goal.EJECTING.defaultRPM);
        io.setHoodAngle(ShooterConstants.kHoodIdleAngleDeg);
      }
    }

    // "Shooter/Goal" and "Shooter/AverageRPM" are logged via @AutoLogOutput on their getter
    // methods — do NOT repeat them here to avoid duplicate-key conflicts in AdvantageKit replay.
    Logger.recordOutput("Shooter/IsReadyToFire", isReadyToFire());
    Logger.recordOutput("Shooter/VelocityErrorRPM", getVelocityErrorRPM());
    Logger.recordOutput("Shooter/TargetHoodAngleDeg", targetHoodAngleDeg);
    Logger.recordOutput("Shooter/CustomRPM", customRPM);
  }

  public void setGoal(Goal goal) {
    this.currentGoal = goal;
  }

  /** Set the flywheel RPM override used by AIM_AND_SHOOT (from the lookup table). */
  public void setCustomRPM(double rpm) {
    this.customRPM = rpm;
  }

  /** Set the hood angle override (degrees). Clamped to hardware limits in periodic(). */
  public void setHoodAngle(double angleDeg) {
    this.targetHoodAngleDeg = angleDeg;
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

  @AutoLogOutput(key = "Shooter/TargetRPM")
  public double getTargetRPM() {
    if (currentGoal == Goal.AIM_AND_SHOOT && customRPM > 0.0) return customRPM;
    return currentGoal.defaultRPM > 0.0 ? currentGoal.defaultRPM : 4500.0;
  }

  /** Current commanded hood angle in degrees (0° = fully down). */
  public double getHoodAngleDeg() {
    return targetHoodAngleDeg;
  }
}
