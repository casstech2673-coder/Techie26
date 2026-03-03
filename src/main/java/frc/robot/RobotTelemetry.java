// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.function.Supplier;

/**
 * Publishes robot state to Shuffleboard each loop.
 *
 * <h2>Robot Status tab</h2>
 *
 * Real-time operational data: angles, RPM, hub distance, ready-to-fire, Field2d.
 *
 * <h2>Tuning tab</h2>
 *
 * LiveWindow-style NetworkTable entries backed by {@link Shooter#tunableManualTurretScale} and
 * friends — edit values in Shuffleboard to tune PID/manual-scale constants without redeploying.
 */
public class RobotTelemetry extends SubsystemBase {

  // ── Data suppliers ─────────────────────────────────────────────────────────
  private final Shooter shooter;
  private final Turret turret;
  private final Intake intake;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Double> hubDistanceSupplier;

  // ── Field2d visualisation ──────────────────────────────────────────────────
  private final Field2d field2d = new Field2d();

  // ── Shuffleboard tabs ──────────────────────────────────────────────────────
  private final ShuffleboardTab statusTab;
  private final ShuffleboardTab tuningTab;

  /**
   * @param shooter Shooter subsystem (RPM, hood, ready-to-fire)
   * @param turret Turret subsystem (angle, aligned flag)
   * @param intake Intake subsystem (arm angle, goal)
   * @param poseSupplier Drive pose supplier ({@code drive::getPose})
   * @param hubDistanceSupplier Distance to the scoring hub in metres
   */
  public RobotTelemetry(
      Shooter shooter,
      Turret turret,
      Intake intake,
      Supplier<Pose2d> poseSupplier,
      Supplier<Double> hubDistanceSupplier) {
    this.shooter = shooter;
    this.turret = turret;
    this.intake = intake;
    this.poseSupplier = poseSupplier;
    this.hubDistanceSupplier = hubDistanceSupplier;

    SmartDashboard.putData("Field", field2d);

    statusTab = Shuffleboard.getTab("Robot Status");
    tuningTab = Shuffleboard.getTab("Tuning");

    buildStatusTab();
    buildTuningTab();
  }

  // ── Tab builders (called once in constructor) ──────────────────────────────

  private void buildStatusTab() {
    // Row 0: Shooter
    statusTab.addDouble("Target RPM", shooter::getTargetRPM).withPosition(0, 0).withSize(2, 1);
    statusTab.addDouble("Actual RPM", shooter::getAverageRPM).withPosition(2, 0).withSize(2, 1);
    statusTab.addDouble("Hood Angle °", shooter::getHoodAngleDeg).withPosition(4, 0).withSize(2, 1);
    statusTab
        .addBoolean("Ready to Fire", shooter::isReadyToFire)
        .withPosition(6, 0)
        .withSize(2, 1)
        .withWidget(BuiltInWidgets.kBooleanBox);

    // Row 1: Turret
    statusTab
        .addDouble("Turret Angle °", turret::getCurrentAngleDeg)
        .withPosition(0, 1)
        .withSize(2, 1);
    statusTab
        .addBoolean("Turret Aligned", turret::isAligned)
        .withPosition(2, 1)
        .withSize(2, 1)
        .withWidget(BuiltInWidgets.kBooleanBox);
    statusTab
        .addBoolean("Turret Homed", turret::hasHomed)
        .withPosition(4, 1)
        .withSize(2, 1)
        .withWidget(BuiltInWidgets.kBooleanBox);

    // Row 2: Intake / arm
    statusTab.addDouble("Arm Angle °", intake::getArmPositionDeg).withPosition(0, 2).withSize(2, 1);
    statusTab
        .addString("Intake Goal", () -> intake.getGoal().name())
        .withPosition(2, 2)
        .withSize(2, 1);
    statusTab
        .addString("Shooter Goal", () -> shooter.getGoal().name())
        .withPosition(4, 2)
        .withSize(2, 1);

    // Row 3: Hub distance
    statusTab
        .addDouble("Hub Distance m", () -> hubDistanceSupplier.get())
        .withPosition(0, 3)
        .withSize(2, 1);

    // Row 3–6: Field2d widget
    statusTab
        .add("Field", field2d)
        .withWidget(BuiltInWidgets.kField)
        .withPosition(2, 3)
        .withSize(6, 4);
  }

  private void buildTuningTab() {
    // Flywheel PID — values are backed by LoggedNetworkNumber in Shooter.java
    // (Shuffleboard reads from NT; changes in NT automatically reach the tunable)
    tuningTab
        .addDouble("Flywheel kS", () -> ShooterConstants.kFlywheelKs)
        .withPosition(0, 0)
        .withSize(2, 1);
    tuningTab
        .addDouble("Flywheel kV", () -> ShooterConstants.kFlywheelKv)
        .withPosition(2, 0)
        .withSize(2, 1);
    tuningTab
        .addDouble("Flywheel kA", () -> ShooterConstants.kFlywheelKa)
        .withPosition(4, 0)
        .withSize(2, 1);
    tuningTab
        .addDouble("Flywheel kP", () -> ShooterConstants.kFlywheelKp)
        .withPosition(6, 0)
        .withSize(2, 1);

    // Turret PID
    tuningTab
        .addDouble("Turret kP", () -> TurretConstants.kTurretKp)
        .withPosition(0, 1)
        .withSize(2, 1);
    tuningTab
        .addDouble("Turret kD", () -> TurretConstants.kTurretKd)
        .withPosition(2, 1)
        .withSize(2, 1);

    // Manual scalars (editable from NT — same keys as LoggedNetworkNumber in Shooter.java)
    tuningTab
        .addDouble("Manual Turret Scale", () -> Shooter.tunableManualTurretScale.get())
        .withPosition(0, 2)
        .withSize(2, 1);
    tuningTab
        .addDouble("Manual Hood Scale", () -> Shooter.tunableManualHoodScale.get())
        .withPosition(2, 2)
        .withSize(2, 1);

    // Startup angle reminder
    tuningTab
        .addDouble("Turret Startup °", () -> TurretConstants.kStartupAngleDeg)
        .withPosition(0, 3)
        .withSize(2, 1);
  }

  // ── Periodic updates ───────────────────────────────────────────────────────

  @Override
  public void periodic() {
    Pose2d robotPose = poseSupplier.get();
    field2d.setRobotPose(robotPose);

    // Draw a "turret aim" marker: a small pose projected 2 m ahead of the turret
    // (visual indicator of where the turret is currently pointing)
    double turretFieldAngleRad =
        robotPose.getRotation().getRadians() + Math.toRadians(turret.getCurrentAngleDeg());
    Pose2d aimMarker =
        new Pose2d(
            new Translation2d(
                robotPose.getX() + 2.0 * Math.cos(turretFieldAngleRad),
                robotPose.getY() + 2.0 * Math.sin(turretFieldAngleRad)),
            new Rotation2d(turretFieldAngleRad));
    field2d.getObject("TurretAim").setPose(aimMarker);
  }
}
