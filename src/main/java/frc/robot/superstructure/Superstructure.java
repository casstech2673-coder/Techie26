// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private static final double kStationaryThresholdMps = 0.3;
  private static final double kShotCycleDurationSec = 0.25;
  private static final double kFlywheelRadiusMeters = 0.051;

  private final Shooter shooter;
  private final Turret turret;
  private final Intake intake;
  private final Vision vision;

  @FunctionalInterface
  public interface ChassisSpeedsSupplier {
    ChassisSpeeds get();
  }

  private final ChassisSpeedsSupplier chassisSpeedsSupplier;
  private final Supplier<Pose2d> robotPoseSupplier;

  private SuperstructureState activeState = SuperstructureState.IDLE;
  private SuperstructureState requestedState = SuperstructureState.IDLE;
  private boolean isSystemReady = false;

  private final Timer shotCycleTimer = new Timer();

  public Superstructure(
      Shooter shooter,
      Turret turret,
      Intake intake,
      Vision vision,
      ChassisSpeedsSupplier chassisSpeedsSupplier,
      Supplier<Pose2d> robotPoseSupplier) {
    this.shooter = shooter;
    this.turret = turret;
    this.intake = intake;
    this.vision = vision;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.robotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void periodic() {
    Vision.TargetResult target = vision.getTargetResult();
    ChassisSpeeds speeds = chassisSpeedsSupplier.get();
    boolean pieceIndexed = intake.hasPieceIndexed();
    boolean turretWrapping = turret.isWrapping();
    boolean shooterReady = shooter.isReadyToFire();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    boolean isStationary = linearSpeed < kStationaryThresholdMps;

    if (requestedState != activeState) {
      activeState = requestedState;
    }

    switch (activeState) {
      case IDLE -> {
        shooter.setGoal(Shooter.Goal.IDLE);
        turret.setGoal(Turret.Goal.STOW);
        intake.setGoal(Intake.Goal.IDLE);
      }
      case INTAKING -> {
        shooter.setGoal(Shooter.Goal.FEEDING);
        turret.setGoal(Turret.Goal.TRACKING);
        if (target.hasTarget()) turret.setTargetAngle(target.targetAngleDeg());
        intake.setGoal(Intake.Goal.INTAKING);
        if (pieceIndexed) activeState = SuperstructureState.PRE_SHOOT;
      }
      case PRE_SHOOT -> {
        shooter.setGoal(Shooter.Goal.SHOOTING);
        turret.setGoal(Turret.Goal.TRACKING);
        if (target.hasTarget()) turret.setTargetAngle(target.targetAngleDeg());
        intake.setGoal(Intake.Goal.JIGGLE);
        shooter.setTurretAligned(turret.isAligned());
        activeState =
            isStationary ? SuperstructureState.STATIONARY_SHOOT : SuperstructureState.MOVING_SHOOT;
      }
      case STATIONARY_SHOOT -> {
        shooter.setGoal(Shooter.Goal.SHOOTING);
        turret.setGoal(Turret.Goal.TRACKING);
        if (target.hasTarget()) turret.setTargetAngle(target.targetAngleDeg());
        intake.setGoal(Intake.Goal.JIGGLE);
        shooter.setTurretAligned(turret.isAligned());
        if (turretWrapping) break;
        if (shooterReady) {
          activeState = SuperstructureState.SHOOTING;
          shotCycleTimer.reset();
          shotCycleTimer.start();
        }
      }
      case MOVING_SHOOT -> {
        double[] movingShot = computeMovingShot(target, speeds);
        shooter.setGoal(Shooter.Goal.SHOOTING_MOVING);
        shooter.setCustomRPM(movingShot[0]);
        turret.setGoal(Turret.Goal.TRACKING);
        turret.setTargetAngle(movingShot[1]);
        intake.setGoal(Intake.Goal.JIGGLE);
        shooter.setTurretAligned(turret.isAligned());
        if (isStationary) {
          activeState = SuperstructureState.STATIONARY_SHOOT;
        } else if (!turretWrapping && shooterReady) {
          activeState = SuperstructureState.SHOOTING;
          shotCycleTimer.reset();
          shotCycleTimer.start();
        }
      }
      case SHOOTING -> {
        shooter.setGoal(Shooter.Goal.SHOOTING);
        turret.setGoal(Turret.Goal.HOLD);
        intake.setGoal(turretWrapping ? Intake.Goal.IDLE : Intake.Goal.KICK);
        if (shotCycleTimer.hasElapsed(kShotCycleDurationSec)) {
          shotCycleTimer.stop();
          activeState = SuperstructureState.IDLE;
        }
      }
      case EJECTING -> {
        shooter.setGoal(Shooter.Goal.EJECTING);
        turret.setGoal(Turret.Goal.HOLD);
        intake.setGoal(Intake.Goal.EJECT);
      }
    }

    isSystemReady =
        shooterReady
            && !turretWrapping
            && pieceIndexed
            && (activeState == SuperstructureState.STATIONARY_SHOOT
                || activeState == SuperstructureState.MOVING_SHOOT
                || activeState == SuperstructureState.SHOOTING);

    Logger.recordOutput("Superstructure/ActiveState", activeState.name());
    Logger.recordOutput("Superstructure/RequestedState", requestedState.name());
    Logger.recordOutput("Superstructure/IsSystemReady", isSystemReady);
    Logger.recordOutput("Superstructure/TurretWrapping", turretWrapping);
    Logger.recordOutput("Superstructure/ShooterReady", shooterReady);
    Logger.recordOutput("Superstructure/PieceIndexed", pieceIndexed);
    Logger.recordOutput("Superstructure/IsStationary", isStationary);
    Logger.recordOutput("Superstructure/LinearSpeedMps", linearSpeed);
    Logger.recordOutput("Superstructure/VisionMode", vision.getActiveMode().name());
  }

  private double[] computeMovingShot(Vision.TargetResult target, ChassisSpeeds speeds) {
    Pose2d robotPose = robotPoseSupplier.get();
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    Translation2d toTarget = vision.getTargetFieldPosition().minus(robotPose.getTranslation());

    double baseOmegaRadS = (Shooter.Goal.SHOOTING.defaultRPM / 60.0) * (2.0 * Math.PI);
    double baseShotSpeed = baseOmegaRadS * kFlywheelRadiusMeters;

    double toTargetNorm = toTarget.getNorm();
    if (toTargetNorm < 0.01) {
      return new double[] {Shooter.Goal.SHOOTING.defaultRPM, target.targetAngleDeg()};
    }

    Translation2d requiredVector = toTarget.times(baseShotSpeed / toTargetNorm);
    Translation2d shooterVector = requiredVector.minus(robotVel);

    double leadAngleField = Math.toDegrees(Math.atan2(shooterVector.getY(), shooterVector.getX()));
    double turretLeadAngle = leadAngleField - robotPose.getRotation().getDegrees();

    double adjustedSpeed = shooterVector.getNorm();
    double adjustedRPM = (adjustedSpeed / kFlywheelRadiusMeters) / (2.0 * Math.PI) * 60.0;

    Logger.recordOutput("Superstructure/MovingShot/AdjustedRPM", adjustedRPM);
    Logger.recordOutput("Superstructure/MovingShot/LeadAngleDeg", turretLeadAngle);

    return new double[] {adjustedRPM, turretLeadAngle};
  }

  public void setGoal(SuperstructureState state) {
    this.requestedState = state;
  }

  @AutoLogOutput(key = "Superstructure/ActiveState")
  public SuperstructureState getActiveState() {
    return activeState;
  }

  @AutoLogOutput(key = "Superstructure/IsSystemReady")
  public boolean isSystemReady() {
    return isSystemReady;
  }
}
