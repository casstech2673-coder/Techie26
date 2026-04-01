package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  // ==========================================================
  // SUBSYSTEMS
  // ==========================================================
  private final SwerveDrive m_swerve  = new SwerveDrive();
  private final Vision      m_vision  = new Vision(m_swerve);
  private final Intake      m_intake  = new Intake();
  private final Hopper      m_hopper  = new Hopper();
  private final Shooter     m_shooter = new Shooter();

  // ==========================================================
  // CONTROLLERS
  // ==========================================================
  private final CommandXboxController m_driverController   = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // Telemetry (kept for dashboard; Superstructure no longer commands motors)
  private final GameManager    m_gameManager    = new GameManager(m_driverController, m_operatorController);
  private final Superstructure m_superStructure = new Superstructure(m_swerve, m_intake, m_hopper, m_shooter, m_gameManager);
  private final RobotTelemetry m_telemetry      = new RobotTelemetry(m_swerve, m_shooter, m_intake, m_hopper, m_superStructure, m_gameManager);

  // Autonomous chooser
  private final SendableChooser<Command> autoChooser;

  // --- Shooter setpoints (D-pad to tune) ---
  private double m_flywheelTarget  = -30.0; // RPS (negative = forward)
  private double m_passFlywheelRps = -40.0;

  // Turret auto-aim toggle (operator A)
  private boolean m_turretAutoAim = false;

  // Track whether a pass command is active (for D-pad context tuning)
  private boolean m_passingActive = false;

  // Jiggle timer (operator Start)
  private final Timer m_jiggleTimer = new Timer();

  // Drive-back start position (used in auto routine)
  private double m_driveBackStartX = 0.0;

  public RobotContainer() {
    // Named commands must be registered before AutoBuilder builds chooser
    registerNamedCommands();

    // ----------------------------------------------------------
    // DEFAULT COMMANDS
    // ----------------------------------------------------------

    // Swerve: teleop drive; driver A = hub-align heading
    m_swerve.setDefaultCommand(
        new TeleopDrive(
            m_swerve,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> true,
            () -> m_driverController.a().getAsBoolean(),   // A: back faces hub
            () -> m_driverController.b().getAsBoolean(),   // B: snap -90°
            () -> m_driverController.x().getAsBoolean(),   // X: snap 90°
            () -> m_driverController.y().getAsBoolean(),   // Y: snap 0°
            () -> computeHubAlignHeadingDeg()              // live hub-align angle
        )
    );

    // Shooter: turret manual/auto; flywheels idle-stopped
    m_shooter.setDefaultCommand(Commands.run(() -> {
        double turretInput = MathUtil.applyDeadband(m_operatorController.getRightX(), 0.1);
        if (turretInput != 0.0) {
            // Right-stick X overrides auto-aim
            m_turretAutoAim = false;
            SmartDashboard.putBoolean("Turret/AutoAim", false);
            m_shooter.runTurretManual(turretInput);
        } else if (m_turretAutoAim) {
            aimTurretAtHub();
        } else {
            m_shooter.runTurretManual(0);
        }
        m_shooter.stopFlywheels();
    }, m_shooter));

    // Hopper: B = reverse (unjam), else stop
    m_hopper.setDefaultCommand(Commands.run(() -> {
        if (m_operatorController.b().getAsBoolean()) {
            m_hopper.reverse();
        } else {
            m_hopper.stop();
        }
    }, m_hopper));

    // Intake pivot: only move when joystick pressed; otherwise hold still
    m_intake.setDefaultCommand(Commands.run(() -> {
        double pivotInput = MathUtil.applyDeadband(m_operatorController.getLeftY(), 0.1);
        if (pivotInput != 0.0) {
            m_intake.setPivotPercent(pivotInput * 0.3);
        } else {
            m_intake.setPivotPercent(0);
        }
    }, m_intake));

    // ----------------------------------------------------------
    // AUTO CHOOSER
    // ----------------------------------------------------------
    if (AutoBuilder.isConfigured()) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = new SendableChooser<>();
      DriverStation.reportWarning(
          "AutoBuilder not configured — auto routines unavailable.", false);
    }

    autoChooser.addOption("Drive Back Then Shoot",
        Commands.sequence(
            Commands.runOnce(() -> m_driveBackStartX = m_swerve.getPose().getX()),
            Commands.run(() -> m_swerve.drive(-0.4, 0, 0, true), m_swerve)
                .until(() -> Math.abs(m_swerve.getPose().getX() - m_driveBackStartX) >= 2.0)
                .finallyDo(() -> m_swerve.drive(0, 0, 0, true)),
            Commands.parallel(
                Commands.run(() -> m_shooter.setFlywheelVelocity(-34.0), m_shooter),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> m_hopper.runSlow(), m_hopper)
                )
            ).withTimeout(10.0)
             .finallyDo(() -> { m_shooter.stopFlywheels(); m_hopper.stop(); })
        )
    );

    SmartDashboard.putData("Select Auto", autoChooser);
    SmartDashboard.putBoolean("Turret/AutoAim",       m_turretAutoAim);
    SmartDashboard.putNumber("Shooter/SpeedSetpoint", m_flywheelTarget);
    SmartDashboard.putNumber("Pass/FlywheelRps",      m_passFlywheelRps);

    configureBindings();
  }

  // ==========================================================
  // ROBOT LIFECYCLE HOOKS
  // ==========================================================

  public void onDisabled() {
    m_intake.setCoastMode();
  }

  public void onEnabled() {
    m_intake.setBrakeMode();
    m_jiggleTimer.stop();
  }

  // ==========================================================
  // BUTTON BINDINGS
  // ==========================================================

  private void configureBindings() {

    // --- DRIVER ---
    // Y: zero gyro heading
    m_driverController.y().onTrue(new InstantCommand(() -> m_swerve.zeroHeading()));
    // (A is handled inside TeleopDrive as hub-align heading snap)

    // --- OPERATOR ---

    // A: toggle turret auto-aim on/off
    m_operatorController.a().onTrue(Commands.runOnce(() -> {
        m_turretAutoAim = !m_turretAutoAim;
        SmartDashboard.putBoolean("Turret/AutoAim", m_turretAutoAim);
    }));

    // X: toggle intake rollers on/off (press once = on, press again = off)
    m_operatorController.x().toggleOnTrue(
        Commands.startEnd(
            () -> m_intake.runRollers(),
            () -> m_intake.stopRollers()
        ).withName("IntakeRollers")
    );

    // Y (hold): shoot at constant speed; hopper spins after 0.5 s
    m_operatorController.y().whileTrue(
        Commands.parallel(
            Commands.run(() -> m_shooter.setFlywheelVelocity(m_flywheelTarget), m_shooter),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.run(() -> m_hopper.runSlow(), m_hopper)
            )
        ).finallyDo(() -> { m_shooter.stopFlywheels(); m_hopper.stop(); })
    );

    // RT: distance-based auto-shoot; hopper after 0.5 s; no need to wait for ready
    m_operatorController.rightTrigger().whileTrue(buildAutoShootCommand());
    // On release: return hood to 0
    m_operatorController.rightTrigger().onFalse(moveHoodToPosition(0.0).withTimeout(4.0));

    // LT: pass — hood to -2 rotations, pass flywheel, hopper after 0.5 s
    m_operatorController.leftTrigger().whileTrue(buildPassCommand());
    // On release: return hood to 0
    m_operatorController.leftTrigger().onFalse(moveHoodToPosition(0.0).withTimeout(4.0));

    // RB: hood zeroing — drive into hard stop at 0.20 duty cycle, then zero encoder
    m_operatorController.rightBumper().onTrue(
        Commands.run(() -> m_shooter.setHoodPercent(0.20), m_shooter)
            .until(() -> m_shooter.getHoodCurrent() >
                SmartDashboard.getNumber("Hood/HomingCurrentThreshold", 15.0))
            .finallyDo(() -> { m_shooter.setHoodPercent(0); m_shooter.zeroHoodEncoder(); })
    );

    // LB: move arm to 0 degrees using absolute encoder at constant -0.3 speed
    m_operatorController.leftBumper().onTrue(
        Commands.run(() -> m_intake.setPivotPercent(-0.3), m_intake)
            .until(() -> Math.abs(m_intake.getAbsoluteDegrees()) < 2.0)
            .withTimeout(5.0)
            .finallyDo(() -> m_intake.setPivotPercent(0))
    );

    // Start: jiggle mode — oscillate arm ±0.15 to settle balls
    m_operatorController.start().whileTrue(
        Commands.run(() -> {
            double elapsed = m_jiggleTimer.get() % IntakeConstants.kArmJigglePeriodSec;
            double power = elapsed < IntakeConstants.kArmJigglePeriodSec / 2 ? 0.15 : -0.15;
            m_intake.setPivotPercent(power);
        }, m_intake)
        .beforeStarting(() -> { m_jiggleTimer.reset(); m_jiggleTimer.start(); })
        .finallyDo(() -> { m_jiggleTimer.stop(); m_intake.setPivotPercent(0); })
    );

    // D-Pad: context-sensitive setpoint tuning
    //   If passing active → tunes pass flywheel RPS
    //   Otherwise        → tunes constant-speed shooter RPS
    m_operatorController.povUp().onTrue(Commands.runOnce(() -> {
        if (m_passingActive) {
            m_passFlywheelRps += 0.25;
            SmartDashboard.putNumber("Pass/FlywheelRps", m_passFlywheelRps);
        } else {
            m_flywheelTarget -= 0.25; // more negative = faster
            SmartDashboard.putNumber("Shooter/SpeedSetpoint", m_flywheelTarget);
        }
    }));
    m_operatorController.povDown().onTrue(Commands.runOnce(() -> {
        if (m_passingActive) {
            m_passFlywheelRps -= 0.25;
            SmartDashboard.putNumber("Pass/FlywheelRps", m_passFlywheelRps);
        } else {
            m_flywheelTarget += 0.25; // less negative = slower
            SmartDashboard.putNumber("Shooter/SpeedSetpoint", m_flywheelTarget);
        }
    }));
  }

  // ==========================================================
  // NAMED COMMAND REGISTRATION (PathPlanner auto paths)
  // ==========================================================

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake",
        Commands.startEnd(
            () -> m_intake.runRollers(),
            () -> m_intake.stopRollers()
        ).withName("Intake"));

    NamedCommands.registerCommand("MoveIntakeArmDown",
        Commands.run(() -> m_intake.setPivotPercent(-0.3), m_intake)
            .until(() -> m_intake.getPivotPosition() <=
                IntakeConstants.kArmDeployedRotations + 0.5)
            .withTimeout(3.0)
            .finallyDo(() -> m_intake.setPivotPercent(0))
            .withName("MoveIntakeArmDown"));

    NamedCommands.registerCommand("AutoShootByDistance", buildAutoShootCommand());
    NamedCommands.registerCommand("AutoAlignTurret",     buildAutoAlignTurretCommand());
    NamedCommands.registerCommand("Pass",                buildPassCommand());
  }

  // ==========================================================
  // COMMAND FACTORIES
  // ==========================================================

  /**
   * Auto-shoot: flywheels + hood from distance lookup table;
   * turret auto-aims if enabled; hopper spins after 0.5 s.
   * Does NOT wait for shooter/hood to be ready before firing hopper.
   */
  private Command buildAutoShootCommand() {
    return Commands.parallel(
        Commands.run(() -> {
            double dist = computeDistToHub();
            m_shooter.setFlywheelVelocity(m_shooter.getInterpolatedFlywheelRps(dist));
            // Hood: constant 0.2 power toward target, stops within 0.1 rot tolerance
            double targetHood = m_shooter.getInterpolatedHoodRotations(dist);
            double hoodError  = targetHood - m_shooter.getHoodRotations();
            if (Math.abs(hoodError) > 0.1) {
                m_shooter.setHoodPercent(0.2 * Math.signum(hoodError));
            } else {
                m_shooter.setHoodPercent(0.0);
            }
            // Turret
            if (m_turretAutoAim) aimTurretAtHub();
            else m_shooter.runTurretManual(0);
        }, m_shooter),
        Commands.sequence(
            Commands.waitSeconds(0.5),
            Commands.run(() -> m_hopper.runSlow(), m_hopper)
        )
    ).finallyDo(() -> { m_shooter.stopFlywheels(); m_hopper.stop(); })
     .withName("AutoShootByDistance");
  }

  /**
   * Pass: hood to -2 rotations at constant 0.2, pass flywheel, hopper after 0.5 s.
   * Turret aims at alliance wall.
   */
  private Command buildPassCommand() {
    return Commands.parallel(
        Commands.run(() -> {
            m_passingActive = true;
            m_shooter.setFlywheelVelocity(m_passFlywheelRps);
            // Hood: constant 0.2 toward -2 rotations
            double hoodError = -2.0 - m_shooter.getHoodRotations();
            if (Math.abs(hoodError) > 0.1) {
                m_shooter.setHoodPercent(0.2 * Math.signum(hoodError));
            } else {
                m_shooter.setHoodPercent(0.0);
            }
            aimTurretAtWall();
        }, m_shooter),
        Commands.sequence(
            Commands.waitSeconds(0.5),
            Commands.run(() -> m_hopper.runSlow(), m_hopper)
        )
    ).finallyDo(() -> {
        m_passingActive = false;
        m_shooter.stopFlywheels();
        m_hopper.stop();
    }).withName("Pass");
  }

  /** Continuously auto-aligns turret to the hub. */
  private Command buildAutoAlignTurretCommand() {
    return Commands.run(() -> aimTurretAtHub(), m_shooter)
        .withName("AutoAlignTurret");
  }

  /**
   * Drives the hood at constant 0.2 duty-cycle toward {@code targetRotations}.
   * Ends automatically once within 0.1 rotations of target.
   */
  private Command moveHoodToPosition(double targetRotations) {
    return Commands.run(() -> {
        double error = targetRotations - m_shooter.getHoodRotations();
        if (Math.abs(error) > 0.1) {
            m_shooter.setHoodPercent(0.2 * Math.signum(error));
        } else {
            m_shooter.setHoodPercent(0.0);
        }
    }, m_shooter)
    .until(() -> Math.abs(targetRotations - m_shooter.getHoodRotations()) <= 0.1);
  }

  // ==========================================================
  // HELPER METHODS
  // ==========================================================

  /**
   * Returns the field heading (degrees) that makes the BACK of the robot face the hub.
   * Used by TeleopDrive when the driver holds A.
   */
  private double computeHubAlignHeadingDeg() {
    var pose = m_swerve.getPose();
    boolean isRed = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    double hubX = isRed ? SuperstructureConstants.kRedHubX : SuperstructureConstants.kBlueHubX;
    double hubY = isRed ? SuperstructureConstants.kRedHubY : SuperstructureConstants.kBlueHubY;
    // Angle from robot to hub = direction the FRONT would need to face.
    // Add 180° so the BACK faces the hub.
    return Math.toDegrees(Math.atan2(hubY - pose.getY(), hubX - pose.getX())) + 180.0;
  }

  /** Computes distance (meters) from the turret pivot to the hub center. */
  private double computeDistToHub() {
    var pose = m_swerve.getPose();
    boolean isRed = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    double turretX = pose.getX()
            - ShooterConstants.kTurretOffsetMeters * Math.cos(pose.getRotation().getRadians());
    double turretY = pose.getY()
            - ShooterConstants.kTurretOffsetMeters * Math.sin(pose.getRotation().getRadians());
    double hubX = isRed ? SuperstructureConstants.kRedHubX : SuperstructureConstants.kBlueHubX;
    double hubY = isRed ? SuperstructureConstants.kRedHubY : SuperstructureConstants.kBlueHubY;
    return Math.hypot(hubX - turretX, hubY - turretY);
  }

  /** Commands the turret to track the hub using field-relative geometry. */
  private void aimTurretAtHub() {
    var pose = m_swerve.getPose();
    boolean isRed = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    double turretX = pose.getX()
            - ShooterConstants.kTurretOffsetMeters * Math.cos(pose.getRotation().getRadians());
    double turretY = pose.getY()
            - ShooterConstants.kTurretOffsetMeters * Math.sin(pose.getRotation().getRadians());
    double hubX = isRed ? SuperstructureConstants.kRedHubX : SuperstructureConstants.kBlueHubX;
    double hubY = isRed ? SuperstructureConstants.kRedHubY : SuperstructureConstants.kBlueHubY;
    Rotation2d fieldAngle  = new Rotation2d(Math.atan2(hubY - turretY, hubX - turretX));
    Rotation2d turretAngle = fieldAngle
            .minus(pose.getRotation())
            .minus(Rotation2d.fromDegrees(180.0));
    m_shooter.setTurretTargetDeg(turretAngle.getDegrees() + 180.0);
  }

  /** Commands the turret to aim at the alliance wall (for passing). */
  private void aimTurretAtWall() {
    var pose = m_swerve.getPose();
    boolean isRed = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    double passBearingDeg  = isRed ? 0.0 : 180.0;
    Rotation2d turretAngle = Rotation2d.fromDegrees(passBearingDeg)
            .minus(pose.getRotation())
            .minus(Rotation2d.fromDegrees(180.0));
    m_shooter.setTurretTargetDeg(turretAngle.getDegrees() + 180.0);
  }

  // ==========================================================
  // AUTONOMOUS
  // ==========================================================

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
