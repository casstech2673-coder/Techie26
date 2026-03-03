// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualOperatorCommand;
import frc.robot.commands.PassCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKrakenFOC;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkFlex;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight4;
import frc.robot.subsystems.vision.VisionIOLumaP1;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.Superstructure.IntakeMode;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * RobotContainer — subsystem instantiation, OI binding, and auto selection.
 *
 * <h2>Driver Controller (port 0)</h2>
 *
 * <pre>
 *  RT            → AimAndShoot (hub tracking shot, on-the-move)
 *  Y             → Pass (Alliance Wall alignment, lob pass)
 *  LT            → Floor Intake (standalone) OR intake-while-shooting modifier
 *  LB            → Agitate modifier (arm oscillation to clear hopper jams)
 *  RT + LT       → Hub Shooting + Floor Pickup
 *  Y  + LT       → Passing      + Floor Pickup
 *  RT + LB       → Hub Shooting + Arm Agitate
 *  Y  + LB       → Passing      + Arm Agitate
 *  A             → Heading lock to 0° while held
 *  X             → X-pattern brake
 *  B             → Reset gyro to 0°
 * </pre>
 *
 * <h2>Operator Controller (port 1)</h2>
 *
 * <pre>
 *  Right Stick X → Turret manual (slow rate, interrupts auto aim when stick > 0.1)
 *  Left Stick Y  → Hood angle manual (slow increment/decrement)
 *  RT            → "Dumb" fixed-RPM shot at kManualShootRPM (no vision)
 *  LT            → Intake roller only (no hopper)
 *  LB            → Hopper/kicker only (feed ball to spinning flywheel)
 *  Y             → Arm pivot up (constant slow rate)
 *  A             → Arm pivot down (constant slow rate)
 * </pre>
 */
public class RobotContainer {

  // ── Subsystems ────────────────────────────────────────────────────────────
  private final Drive drive;
  private final Shooter shooter;
  private final Turret turret;
  private final Intake intake;
  private final Vision vision;
  private final Superstructure superstructure;

  @SuppressWarnings("unused")
  private final RobotTelemetry telemetry;

  // ── Controllers ───────────────────────────────────────────────────────────
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // ── Dashboard ─────────────────────────────────────────────────────────────
  private final LoggedDashboardChooser<Command> autoChooser;

  // ── Constructor ───────────────────────────────────────────────────────────
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        shooter = new Shooter(new ShooterIOKrakenFOC());
        turret = new Turret(new TurretIOSparkFlex());
        intake = new Intake(new IntakeIOSparkMax());
        // *** UPDATE these Transform3d values to match the actual camera mounting positions. ***
        vision =
            new Vision(
                new VisionIOLumaP1(
                    "luma-a",
                    new Transform3d(new Translation3d(0.3, 0.2, 0.5), new Rotation3d(0, -0.3, 0))),
                new VisionIOLumaP1(
                    "luma-b",
                    new Transform3d(new Translation3d(0.3, -0.2, 0.5), new Rotation3d(0, -0.3, 0))),
                new VisionIOLimelight4("limelight"));
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        shooter = new Shooter(new ShooterIOSim());
        turret = new Turret(new TurretIOSim());
        intake = new Intake(new IntakeIOSim());
        vision = new Vision(new VisionIO() {});
        break;

      default:
        // Replay — all IO implementations are no-ops
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new Shooter(new ShooterIO() {});
        turret = new Turret(new TurretIO() {});
        intake = new Intake(new IntakeIO() {});
        vision = new Vision(new VisionIO() {});
        break;
    }

    // Cross-wire vision → drive pose estimator
    vision.setPoseConsumer(drive::addVisionMeasurement);
    vision.setTurretAngleSupplier(turret::getCurrentAngleDeg);
    // Provide fused odometry pose to Vision so FIELD_TRACKING uses the most stable estimate.
    vision.setRobotPoseSupplier(drive::getPose);

    // Build the Superstructure (owns shooter + turret + intake, coordinates with vision + drive)
    superstructure =
        new Superstructure(
            shooter, turret, intake, vision, drive::getChassisSpeeds, drive::getPose);

    // Shuffleboard dashboard (registered as a subsystem so periodic() runs automatically)
    telemetry =
        new RobotTelemetry(
            shooter,
            turret,
            intake,
            drive::getPose,
            () -> {
              var hub = FieldConstants.getHubForAlliance();
              return drive.getPose().getTranslation().getDistance(hub);
            });

    // Auto chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
    configureOperatorBindings();
  }

  /**
   * Button / trigger binding configuration.
   *
   * <p>Design: Superstructure is required by AimAndShootCommand, PassCommand, and IntakeCommand.
   * Intake-mode modifiers (LT and LB while shooting/passing) call {@link
   * Superstructure#setIntakeMode} via no-requirement runEnd lambdas so they can run concurrently
   * with the active shooter command.
   */
  private void configureButtonBindings() {

    // ── Drive defaults ────────────────────────────────────────────────────
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // ── Shooter / Pass triggers ───────────────────────────────────────────
    Trigger rt = controller.rightTrigger(0.5);
    Trigger lt = controller.leftTrigger(0.5);
    Trigger lb = controller.leftBumper();
    Trigger y = controller.y();

    // Right Trigger → unified AimAndShoot (hub tracking, on-the-move).
    // Runs as long as RT is held; both stationary and moving shots handled internally.
    rt.whileTrue(new AimAndShootCommand(superstructure));

    // Y button → Alliance Wall pass mode.
    // Turret locks to -180° field-relative; pass RPM + hood angle applied.
    y.whileTrue(new PassCommand(superstructure));

    // ── Floor Intake (Left Trigger) ───────────────────────────────────────

    // LT alone (no shooter active) → floor pickup, auto-advances to AIM_AND_SHOOT.
    lt.and(rt.negate()).and(y.negate()).whileTrue(new IntakeCommand(superstructure));

    // LT + RT → Hub Shooting + Floor Pickup (Global State 1)
    rt.and(lt)
        .whileTrue(
            Commands.runEnd(
                () -> superstructure.setIntakeMode(IntakeMode.FLOOR_INTAKE),
                () -> superstructure.setIntakeMode(IntakeMode.IDLE)));

    // LT + Y → Passing + Floor Pickup (Global State 2)
    y.and(lt)
        .whileTrue(
            Commands.runEnd(
                () -> superstructure.setIntakeMode(IntakeMode.FLOOR_INTAKE),
                () -> superstructure.setIntakeMode(IntakeMode.IDLE)));

    // ── Agitate (Left Bumper) ─────────────────────────────────────────────
    // Active only when Shooter or Pass is running.
    // Arm oscillates up/down to prevent hopper jams.

    // LB + RT → Hub Shooting + Arm Agitate (Global State 3)
    rt.and(lb)
        .whileTrue(
            Commands.runEnd(
                () -> superstructure.setIntakeMode(IntakeMode.AGITATE),
                () -> superstructure.setIntakeMode(IntakeMode.IDLE)));

    // LB + Y → Passing + Arm Agitate (Global State 4)
    y.and(lb)
        .whileTrue(
            Commands.runEnd(
                () -> superstructure.setIntakeMode(IntakeMode.AGITATE),
                () -> superstructure.setIntakeMode(IntakeMode.IDLE)));
  }

  /**
   * Operator overrides (port 1).
   *
   * <p>Turret/hood sticks and operator RT all require Superstructure → they interrupt any active
   * AimAndShootCommand or PassCommand. Intake buttons (LT/LB/Y/A) are no-requirement runEnd lambdas
   * that set IntakeMode, identical to the driver's agitate/floor-intake pattern.
   */
  private void configureOperatorBindings() {

    // ── Turret + Hood + Manual Shoot ──────────────────────────────────────
    // Fires when any meaningful operator input is detected (deadband 0.1 on sticks, 0.5 on RT).
    // Schedules ManualOperatorCommand, which requires Superstructure and cancels auto commands.
    Trigger opManualActive =
        new Trigger(
            () ->
                Math.abs(MathUtil.applyDeadband(operatorController.getRightX(), 0.1)) > 0.0
                    || Math.abs(MathUtil.applyDeadband(operatorController.getLeftY(), 0.1)) > 0.0
                    || operatorController.getRightTriggerAxis() > 0.5);

    opManualActive.whileTrue(
        new ManualOperatorCommand(
            superstructure,
            () -> operatorController.getRightX(),
            () -> -operatorController.getLeftY(),
            () -> operatorController.getRightTriggerAxis() > 0.5));

    // ── Intake roller (Operator LT) ───────────────────────────────────────
    operatorController
        .leftTrigger(0.5)
        .whileTrue(
            Commands.runEnd(
                () -> superstructure.setIntakeMode(IntakeMode.MANUAL_ROLLER),
                () -> superstructure.setIntakeMode(IntakeMode.IDLE)));

    // ── Hopper/kicker (Operator LB) ───────────────────────────────────────
    operatorController
        .leftBumper()
        .whileTrue(
            Commands.runEnd(
                () -> superstructure.setIntakeMode(IntakeMode.MANUAL_HOPPER),
                () -> superstructure.setIntakeMode(IntakeMode.IDLE)));

    // ── Arm pivot up (Operator Y) ─────────────────────────────────────────
    operatorController
        .y()
        .whileTrue(
            Commands.runEnd(
                () -> superstructure.setIntakeMode(IntakeMode.MANUAL_ARM_UP),
                () -> superstructure.setIntakeMode(IntakeMode.IDLE)));

    // ── Arm pivot down (Operator A) ───────────────────────────────────────
    operatorController
        .a()
        .whileTrue(
            Commands.runEnd(
                () -> superstructure.setIntakeMode(IntakeMode.MANUAL_ARM_DOWN),
                () -> superstructure.setIntakeMode(IntakeMode.IDLE)));
  }

  /** Returns the autonomous command selected on the dashboard. */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
