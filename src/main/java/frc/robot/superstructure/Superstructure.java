// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure — the robot's central "Air Traffic Controller".
 *
 * <p>Owns the top-level state machine. All shooter / turret / intake logic lives here; commands are
 * thin wrappers that set goals and intake modes, nothing more.
 *
 * <p>Two independent axes of control are managed:
 *
 * <ul>
 *   <li>{@link SuperstructureState} — what the shooter + turret are doing.
 *   <li>{@link IntakeMode} — what the intake / hopper system is doing while shooting or passing.
 * </ul>
 *
 * <h2>Alliance-awareness</h2>
 *
 * <p>Each loop, {@link FieldConstants#getHubForAlliance()} and {@link
 * FieldConstants#getPassFieldBearingDeg()} are queried so the correct hub and pass direction are
 * always used, including mid-match FMS alliance assignment changes.
 */
public class Superstructure extends SubsystemBase {

  // ── Tuning ────────────────────────────────────────────────────────────────
  private static final double kShotCycleDurationSec = 0.25;

  // ── Intake Mode (independent of shooter goal) ─────────────────────────────
  /**
   * Controls what the intake / hopper does while AIM_AND_SHOOT or PASSING is active.
   *
   * <ul>
   *   <li>IDLE — hopper stopped, arm at 0°.
   *   <li>FLOOR_INTAKE — intake roller and hopper run (floor pickup).
   *   <li>AGITATE — intake arm oscillates to clear jams; no floor pickup.
   * </ul>
   *
   * <p>AGITATE only has effect when shooterGoal is AIM_AND_SHOOT or PASSING. Binding: Left Bumper.
   */
  public enum IntakeMode {
    IDLE,
    FLOOR_INTAKE,
    AGITATE,
    /** Operator LT: intake roller only, no hopper. */
    MANUAL_ROLLER,
    /** Operator LB: hopper/kicker only, no roller. */
    MANUAL_HOPPER,
    /** Operator Y: arm pivots upward at kArmManualDegPerSec. */
    MANUAL_ARM_UP,
    /** Operator A: arm pivots downward at kArmManualDegPerSec. */
    MANUAL_ARM_DOWN
  }

  // ── Subsystems ────────────────────────────────────────────────────────────
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

  // ── State Machine ─────────────────────────────────────────────────────────
  private SuperstructureState activeState = SuperstructureState.IDLE;
  private SuperstructureState requestedState = SuperstructureState.IDLE;
  private IntakeMode activeIntakeMode = IntakeMode.IDLE;

  // Tracks which shooting mode was active before SHOOTING so we can return correctly.
  private SuperstructureState preShootingState = SuperstructureState.IDLE;

  private boolean isSystemReady = false;
  private final Timer shotCycleTimer = new Timer();

  // ── Manual Operator Inputs ────────────────────────────────────────────────
  // Set each loop by ManualOperatorCommand; consumed in MANUAL_CONTROL case.
  private double manualTurretDeltaDeg = 0.0;
  private double manualHoodAngleDeg = ShooterConstants.kHoodIdleAngleDeg;
  private double manualFlywheelRPM = 0.0;

  // ── Lookup Tables ─────────────────────────────────────────────────────────
  private final InterpolatingDoubleTreeMap rpmTable = ShooterConstants.buildRPMTable();
  private final InterpolatingDoubleTreeMap hoodTable = ShooterConstants.buildHoodTable();

  // ── Constructor ───────────────────────────────────────────────────────────
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

  // ── Main Loop ─────────────────────────────────────────────────────────────
  @Override
  public void periodic() {
    // Alliance-aware hub and pass bearing — refreshed every loop so a mid-match FMS
    // alliance assignment is picked up without a restart.
    Translation2d hubPosition = FieldConstants.getHubForAlliance();
    double passFieldBearingDeg = FieldConstants.getPassFieldBearingDeg();

    // Keep Vision's target field position in sync with the current alliance hub.
    vision.setTargetFieldPosition(hubPosition);

    Vision.TargetResult target = vision.getTargetResult();
    ChassisSpeeds speeds = chassisSpeedsSupplier.get();
    Pose2d robotPose = robotPoseSupplier.get();
    boolean turretWrapping = turret.isWrapping();
    boolean shooterReady = shooter.isReadyToFire();

    // Apply requested state transitions.
    // Guard: never interrupt a mid-shot cycle.
    if (requestedState != activeState && activeState != SuperstructureState.SHOOTING) {
      activeState = requestedState;
    }

    switch (activeState) {

        // ── IDLE ─────────────────────────────────────────────────────────────
      case IDLE -> {
        shooter.setGoal(Shooter.Goal.IDLE);
        turret.setGoal(Turret.Goal.STOW);
        // Route through IntakeMode so operator LT/LB/Y/A work even when not shooting.
        applyIntakeMode(false);
      }

        // ── INTAKING ─────────────────────────────────────────────────────────
      case INTAKING -> {
        shooter.setGoal(Shooter.Goal.FEEDING);
        turret.setGoal(Turret.Goal.TRACKING);
        if (target.hasTarget()) turret.setTargetAngle(target.targetAngleDeg());
        intake.setGoal(Intake.Goal.INTAKING);
      }

        // ── AIM_AND_SHOOT ─────────────────────────────────────────────────────
      case AIM_AND_SHOOT -> {
        double[] shot = computeAimAndShot(target, speeds, robotPose, hubPosition);
        double adjustedRPM = shot[0];
        double leadAngleDeg = shot[1];
        double hoodAngleDeg = shot[2];

        shooter.setGoal(Shooter.Goal.AIM_AND_SHOOT);
        shooter.setCustomRPM(adjustedRPM);
        shooter.setHoodAngle(hoodAngleDeg);
        turret.setGoal(Turret.Goal.TRACKING);
        turret.setTargetAngle(leadAngleDeg);
        shooter.setTurretAligned(turret.isAligned());

        applyIntakeMode(turretWrapping);

        if (!turretWrapping && shooterReady) {
          preShootingState = SuperstructureState.AIM_AND_SHOOT;
          activeState = SuperstructureState.SHOOTING;
          shotCycleTimer.reset();
          shotCycleTimer.start();
        }
      }

        // ── PASSING ───────────────────────────────────────────────────────────
      case PASSING -> {
        // Convert field-relative pass bearing to robot-relative turret angle:
        //   turretAngle = fieldBearing − robotHeading
        // Blue:  passFieldBearingDeg = 180° → face blue alliance wall (−X direction)
        // Red:   passFieldBearingDeg =  0°  → face red  alliance wall (+X direction)
        double passAngleDeg = passFieldBearingDeg - robotPose.getRotation().getDegrees();
        shooter.setGoal(Shooter.Goal.PASSING);
        turret.setGoal(Turret.Goal.TRACKING);
        turret.setTargetAngle(passAngleDeg);
        shooter.setTurretAligned(turret.isAligned());

        applyIntakeMode(turretWrapping);

        if (!turretWrapping && shooterReady) {
          preShootingState = SuperstructureState.PASSING;
          activeState = SuperstructureState.SHOOTING;
          shotCycleTimer.reset();
          shotCycleTimer.start();
        }
      }

        // ── SHOOTING ─────────────────────────────────────────────────────────
      case SHOOTING -> {
        if (preShootingState == SuperstructureState.PASSING) {
          shooter.setGoal(Shooter.Goal.PASSING);
        } else {
          shooter.setGoal(Shooter.Goal.AIM_AND_SHOOT);
        }
        turret.setGoal(Turret.Goal.HOLD);
        intake.setGoal(turretWrapping ? Intake.Goal.IDLE : Intake.Goal.KICK);

        if (shotCycleTimer.hasElapsed(kShotCycleDurationSec)) {
          shotCycleTimer.stop();
          activeState =
              (requestedState == SuperstructureState.AIM_AND_SHOOT
                      || requestedState == SuperstructureState.PASSING)
                  ? preShootingState
                  : SuperstructureState.IDLE;
        }
      }

        // ── MANUAL_CONTROL ───────────────────────────────────────────────────
      case MANUAL_CONTROL -> {
        // Turret: clamp incremental delta to soft limits so hardware is protected.
        double newTurretAngle =
            MathUtil.clamp(
                turret.getCurrentAngleDeg() + manualTurretDeltaDeg,
                TurretConstants.kSoftMinAngleDeg,
                TurretConstants.kSoftMaxAngleDeg);
        turret.setGoal(Turret.Goal.TRACKING);
        turret.setTargetAngle(newTurretAngle);

        if (manualFlywheelRPM > 0.0) {
          shooter.setGoal(Shooter.Goal.AIM_AND_SHOOT);
          shooter.setCustomRPM(manualFlywheelRPM);
          shooter.setTurretAligned(true); // bypass alignment gate — operator controls timing
        } else {
          shooter.setGoal(Shooter.Goal.IDLE);
        }
        shooter.setHoodAngle(
            MathUtil.clamp(
                manualHoodAngleDeg,
                ShooterConstants.kHoodMinAngleDeg,
                ShooterConstants.kHoodMaxAngleDeg));

        applyIntakeMode(turretWrapping);
      }

        // ── EJECTING ─────────────────────────────────────────────────────────
      case EJECTING -> {
        shooter.setGoal(Shooter.Goal.EJECTING);
        turret.setGoal(Turret.Goal.HOLD);
        intake.setGoal(Intake.Goal.EJECT);
      }
    }

    isSystemReady =
        shooterReady
            && !turretWrapping
            && (activeState == SuperstructureState.AIM_AND_SHOOT
                || activeState == SuperstructureState.PASSING
                || activeState == SuperstructureState.SHOOTING);

    // Only log keys NOT already covered by @AutoLogOutput on a getter method,
    // to avoid duplicate-key conflicts in AdvantageKit replay.
    Logger.recordOutput("Superstructure/RequestedState", requestedState.name());
    Logger.recordOutput("Superstructure/IntakeMode", activeIntakeMode.name());
    Logger.recordOutput("Superstructure/TurretWrapping", turretWrapping);
    Logger.recordOutput("Superstructure/ShooterReady", shooterReady);
    Logger.recordOutput("Superstructure/VisionMode", vision.getActiveMode().name());
    Logger.recordOutput("Superstructure/HubPositionX", hubPosition.getX());
    Logger.recordOutput("Superstructure/HubPositionY", hubPosition.getY());
    Logger.recordOutput("Superstructure/PassBearingDeg", passFieldBearingDeg);
  }

  // ── Internal Helpers ──────────────────────────────────────────────────────

  /**
   * Unified "On-The-Move" shot computation.
   *
   * <p>Returns [adjustedRPM, turretLeadAngleDeg, hoodAngleDeg].
   *
   * <ol>
   *   <li><b>Optional TOF position prediction</b> ({@link
   *       ShooterConstants#kEnableTimeOfFlightPrediction}): iteratively estimates where the robot
   *       will be when the ball reaches the hub and uses that predicted distance for RPM/hood
   *       lookups. This improves accuracy when driving radially toward/away from the hub.
   *   <li><b>Moving-shot vector math</b>: robot chassis velocity is subtracted from the required
   *       ball velocity to produce the shooter vector (lead angle + RPM scaling). When stationary
   *       (v ≈ 0) this degrades cleanly to a direct-aim shot.
   *   <li><b>Hood angle correction</b>: the hood angle from the lookup table assumes a stationary
   *       robot. When moving, the flywheel produces a different speed in the robot frame, changing
   *       the vertical launch component. The angle is corrected via:
   *       {@code sin(θ_adj) = sin(θ_base) × (baseBallSpeed / adjustedBallSpeed)}.
   * </ol>
   */
  private double[] computeAimAndShot(
      Vision.TargetResult target,
      ChassisSpeeds speeds,
      Pose2d robotPose,
      Translation2d hubPosition) {

    // Turret pivot position on the field.
    // Accounts for the physical offset between robot center and turret pivot.
    Translation2d turretCenter =
        robotPose
            .getTranslation()
            .plus(TurretConstants.kTurretOffsetFromRobotCenter.rotateBy(robotPose.getRotation()));

    Translation2d toTarget = hubPosition.minus(turretCenter);

    // Distance: prefer direct Limelight measurement; fall back to pose-estimated distance.
    double distance = target.hasTarget() ? target.distanceToTargetMeters() : toTarget.getNorm();
    distance = Math.max(distance, 0.5); // guard against degenerate zero

    Translation2d robotVelVec =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    // ── Optional TOF Position Prediction ──────────────────────────────────────────────────────
    // Iteratively predict where the robot will be when the ball reaches the hub (using ball
    // flight time as a proxy for spin-up + flight latency). The resulting `lookupDist` drives
    // RPM and base hood angle lookups so the robot pre-selects the right parameters for its
    // future position — most useful when closing in on or backing away from the hub.
    //
    // The aim *direction* always uses the actual current turret position so the ball is physically
    // aimed at the hub from where it is launched. Only the table-lookup distance is adjusted.
    double lookupDist = distance;
    if (ShooterConstants.kEnableTimeOfFlightPrediction) {
      for (int i = 0; i < ShooterConstants.kTofIterations; i++) {
        lookupDist = Math.max(lookupDist, 0.5);
        double iterRPM = rpmTable.get(lookupDist);
        double iterSpeed =
            (iterRPM / 60.0) * (2.0 * Math.PI) * ShooterConstants.kFlywheelRadiusMeters;
        double flightTime = lookupDist / iterSpeed;
        Translation2d predictedTurretCenter = turretCenter.plus(robotVelVec.times(flightTime));
        lookupDist = Math.max(hubPosition.minus(predictedTurretCenter).getNorm(), 0.5);
      }
    }

    double baseRPM = rpmTable.get(lookupDist);
    double hoodAngleDeg = hoodTable.get(lookupDist);
    double baseOmega = (baseRPM / 60.0) * (2.0 * Math.PI);
    double baseBallSpeed = baseOmega * ShooterConstants.kFlywheelRadiusMeters;

    if (toTarget.getNorm() < 0.01) {
      // Turret is at (or extremely close to) the hub — aim straight ahead as fallback.
      return new double[] {baseRPM, target.targetAngleDeg(), hoodAngleDeg};
    }

    // ── Moving-Shot Vector Math ────────────────────────────────────────────────────────────────
    // Aim direction uses the actual current turret→hub vector so the ball launches toward the hub.
    Translation2d requiredVel = toTarget.times(baseBallSpeed / toTarget.getNorm());
    Translation2d shooterVector = requiredVel.minus(robotVelVec);

    // Lead angle: field-relative direction of the shooter vector → convert to robot-relative.
    double leadAngleField = Math.toDegrees(Math.atan2(shooterVector.getY(), shooterVector.getX()));
    double turretLeadAngle = leadAngleField - robotPose.getRotation().getDegrees();

    double adjustedBallSpeed = shooterVector.getNorm();
    double adjustedRPM =
        (adjustedBallSpeed / ShooterConstants.kFlywheelRadiusMeters) / (2.0 * Math.PI) * 60.0;

    // ── Hood Angle Correction for Moving Shots ─────────────────────────────────────────────────
    // The hood angle controls the vertical/horizontal split of ball velocity.
    // The flywheel produces `adjustedBallSpeed` in the robot frame. The ball's field-frame
    // horizontal speed toward the hub is always `baseBallSpeed` (from the vector math above),
    // but the vertical component scales with `adjustedBallSpeed`. So the required launch angle θ
    // satisfies:  sin(θ_adj) = sin(θ_base) × (baseBallSpeed / adjustedBallSpeed)
    //
    // Derivation: for a target at the same height, the vertical impulse needed is fixed
    // (gravity × flight time). Since flight time = distance / baseBallSpeed (constant),
    // the required vertical velocity = adjustedBallSpeed × sin(θ_adj) = fixed constant.
    // ∴  sin(θ_adj) = sin(θ_base) × baseBallSpeed / adjustedBallSpeed.
    if (adjustedBallSpeed > 0.01) {
      double sinBase = Math.sin(Math.toRadians(hoodAngleDeg));
      double sinAdjusted =
          MathUtil.clamp(sinBase * (baseBallSpeed / adjustedBallSpeed), -1.0, 1.0);
      hoodAngleDeg = Math.toDegrees(Math.asin(sinAdjusted));
    }

    Logger.recordOutput("Superstructure/AimAndShot/DistanceM", distance);
    Logger.recordOutput("Superstructure/AimAndShot/LookupDistM", lookupDist);
    Logger.recordOutput("Superstructure/AimAndShot/TofEnabled", ShooterConstants.kEnableTimeOfFlightPrediction);
    Logger.recordOutput("Superstructure/AimAndShot/BaseRPM", baseRPM);
    Logger.recordOutput("Superstructure/AimAndShot/AdjustedRPM", adjustedRPM);
    Logger.recordOutput("Superstructure/AimAndShot/BaseBallSpeedMps", baseBallSpeed);
    Logger.recordOutput("Superstructure/AimAndShot/AdjustedBallSpeedMps", adjustedBallSpeed);
    Logger.recordOutput("Superstructure/AimAndShot/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("Superstructure/AimAndShot/LeadAngleDeg", turretLeadAngle);

    return new double[] {adjustedRPM, turretLeadAngle, hoodAngleDeg};
  }

  /**
   * Routes intake goal based on the active IntakeMode while shooting/passing.
   *
   * <p>If the turret is mid-wrap, intake is silenced to avoid feeding during the crossover.
   */
  private void applyIntakeMode(boolean turretWrapping) {
    if (turretWrapping) {
      intake.setGoal(Intake.Goal.IDLE);
      return;
    }
    switch (activeIntakeMode) {
      case FLOOR_INTAKE -> intake.setGoal(Intake.Goal.INTAKING);
      case AGITATE -> intake.setGoal(Intake.Goal.AGITATE);
      case MANUAL_ROLLER -> intake.setGoal(Intake.Goal.ROLLER_ONLY);
      case MANUAL_HOPPER -> intake.setGoal(Intake.Goal.KICK);
      case MANUAL_ARM_UP -> intake.setGoal(Intake.Goal.MANUAL_ARM_UP);
      case MANUAL_ARM_DOWN -> intake.setGoal(Intake.Goal.MANUAL_ARM_DOWN);
      default -> intake.setGoal(Intake.Goal.IDLE);
    }
  }

  // ── Public API ────────────────────────────────────────────────────────────

  /**
   * Request a new shooter / turret state. Called by AimAndShootCommand, PassCommand, IntakeCommand.
   */
  public void setGoal(SuperstructureState state) {
    this.requestedState = state;
  }

  /**
   * Set the intake mode while AIM_AND_SHOOT or PASSING is active. Called independently of {@link
   * #setGoal} so shooter and intake bindings can be composed freely in RobotContainer without
   * subsystem requirement conflicts.
   */
  public void setIntakeMode(IntakeMode mode) {
    this.activeIntakeMode = mode;
  }

  /**
   * Pass operator manual inputs to the superstructure for use in MANUAL_CONTROL state. Called each
   * loop by ManualOperatorCommand.
   *
   * @param turretDeltaDeg Turret angle increment this loop (degrees). Soft limits enforced.
   * @param hoodAngleDeg Absolute hood angle setpoint (degrees). Clamped to [min, max].
   * @param flywheelRPM Flywheel setpoint (RPM). 0 = stop flywheel.
   */
  public void setManualInputs(double turretDeltaDeg, double hoodAngleDeg, double flywheelRPM) {
    this.manualTurretDeltaDeg = turretDeltaDeg;
    this.manualHoodAngleDeg = hoodAngleDeg;
    this.manualFlywheelRPM = flywheelRPM;
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
