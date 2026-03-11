package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final Vision      m_vision  = new Vision(m_swerve); // MegaTag2 + Arducam → fused odometry
  private final Intake      m_intake  = new Intake();
  private final Hopper      m_hopper  = new Hopper();
  private final Shooter     m_shooter = new Shooter();

  // ==========================================================
  // CONTROLLERS
  // ==========================================================
  private final CommandXboxController m_driverController   = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // Autonomous chooser
  private final SendableChooser<Command> autoChooser;

  // --- Adjustable setpoints (D-pad to tune) ---
  private double m_flywheelTarget    = -10.0; // RPS (negative = forward)
  private double m_hoodTarget        = 0.0;   // motor rotations from home
  private double m_passFlywheelRps   = 30.0;  // TODO: tune on real robot
  private double m_passHoodRotations = 1.5;   // TODO: tune on real robot

  // Turret auto-aim: false = manual right-stick, true = hub tracking
  private boolean m_turretAutoAim = false;

  // --- Arm mode state machine (driver Start cycles) ---
  private enum ArmMode { ZERO, JIGGLE, MANUAL }
  private ArmMode m_armMode   = ArmMode.ZERO;
  private final Timer m_jiggleTimer = new Timer();

  // Set true while the RB passing command is active — D-pad tunes pass setpoints
  private boolean m_passingActive = false;

  public RobotContainer() {
    // Drivetrain default command
    m_swerve.setDefaultCommand(
        new TeleopDrive(
            m_swerve,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> true,
            () -> m_driverController.a().getAsBoolean(), // A: snap 0°
            () -> m_driverController.b().getAsBoolean(), // B: snap -90°
            () -> m_driverController.x().getAsBoolean(), // X: snap 90°
            () -> m_driverController.y().getAsBoolean()  // Y: snap 180°
        )
    );

    if (AutoBuilder.isConfigured()) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = new SendableChooser<>();
      DriverStation.reportWarning(
          "AutoBuilder not configured — auto routines unavailable. "
          + "Open PathPlanner GUI, configure the robot, and redeploy.", false);
    }
    SmartDashboard.putData("Select Auto", autoChooser);

    // Initial SmartDashboard values
    SmartDashboard.putBoolean("Turret/AutoAim",        m_turretAutoAim);
    SmartDashboard.putNumber("Shooter/SpeedSetpoint",  m_flywheelTarget);
    SmartDashboard.putNumber("Pass/FlywheelRps",       m_passFlywheelRps);
    SmartDashboard.putNumber("Pass/HoodRotations",     m_passHoodRotations);

    configureBindings();
  }

  // ==========================================================
  // ROBOT LIFECYCLE HOOKS (called from Robot.java)
  // ==========================================================

  /** Called in disabledInit — releases the arm pivot so it can be moved freely by hand. */
  public void onDisabled() {
    m_intake.setCoastMode();
  }

  /** Called in autonomousInit and teleopInit — locks the arm and resets to ZERO mode. */
  public void onEnabled() {
    m_intake.setBrakeMode();
    m_armMode = ArmMode.ZERO;
    m_jiggleTimer.stop();
  }

  private void configureBindings() {

    // ==========================================================
    // DRIVER BUTTONS
    // ==========================================================

    // Back: zero gyro heading
    m_driverController.back().onTrue(Commands.runOnce(() -> m_swerve.zeroHeading()));

    // Start: cycle arm mode
    //   MANUAL → ZERO      (re-engage auto-hold-zero after manual override)
    //   ZERO   → JIGGLE    (start oscillating to help balls settle)
    //   JIGGLE → ZERO      (stop jiggling, return to zero)
    m_driverController.start().onTrue(Commands.runOnce(() -> {
        if (m_armMode == ArmMode.MANUAL) {
            m_armMode = ArmMode.ZERO;
            m_jiggleTimer.stop();
        } else if (m_armMode == ArmMode.ZERO) {
            m_armMode = ArmMode.JIGGLE;
            m_jiggleTimer.reset();
            m_jiggleTimer.start();
        } else { // JIGGLE → ZERO
            m_armMode = ArmMode.ZERO;
            m_jiggleTimer.stop();
        }
    }));

    // LB: intake rollers only (arm default command keeps running concurrently)
    m_driverController.leftBumper().whileTrue(
        Commands.startEnd(
            () -> m_intake.runRollers(),
            () -> m_intake.stopRollers()
        )
    );

    // LT: DUMB VACUUM — turret fixed at 180° (home), fixed flywheel+hood (D-pad to tune)
    m_driverController.leftTrigger().whileTrue(
        Commands.run(() -> {
            m_shooter.setTurretTargetDeg(180.0);
            m_shooter.setFlywheelVelocity(m_flywheelTarget);
            m_shooter.setHoodPosition(m_hoodTarget);
            m_intake.runRollers();
            if (m_shooter.isReadyToFire()) {
                m_hopper.startFeedSequence();
            } else {
                m_hopper.stop();
            }
        }, m_shooter, m_hopper)
        .finallyDo(() -> {
            m_shooter.stopFlywheels();
            m_hopper.stop();
            m_intake.stopRollers();
        })
    );

    // RB: PASSING — turret locks to alliance wall bearing, pass setpoints (D-pad to tune)
    m_driverController.rightBumper().whileTrue(
        Commands.run(() -> {
            m_passingActive = true;

            var pose = m_swerve.getPose();
            boolean isRed = DriverStation.getAlliance()
                    .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

            // Fixed global bearing toward our alliance wall (Blue 180°, Red 0°)
            double passBearingDeg = isRed ? 0.0 : 180.0;
            Rotation2d turretPassAngle = Rotation2d.fromDegrees(passBearingDeg)
                    .minus(pose.getRotation())
                    .minus(Rotation2d.fromDegrees(180.0)); // subtract home offset
            m_shooter.setTurretTargetDeg(turretPassAngle.getDegrees() + 180.0);

            m_shooter.setFlywheelVelocity(m_passFlywheelRps);
            m_shooter.setHoodPosition(m_passHoodRotations);
            m_intake.runRollers();
            if (m_shooter.isReadyToFire()) {
                m_hopper.startFeedSequence();
            } else {
                m_hopper.stop();
            }
        }, m_shooter, m_hopper)
        .finallyDo(() -> {
            m_passingActive = false;
            m_shooter.stopFlywheels();
            m_hopper.stop();
            m_intake.stopRollers();
        })
    );

    // RT: VACUUM AUTO — distance-based flywheel+hood, turret tracks hub (when auto-aim on)
    m_driverController.rightTrigger().whileTrue(
        Commands.run(() -> {
            var pose = m_swerve.getPose();
            boolean isRed = DriverStation.getAlliance()
                    .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

            // Turret's real field position (7.25" behind robot center)
            double turretX = pose.getX()
                    - ShooterConstants.kTurretOffsetMeters * Math.cos(pose.getRotation().getRadians());
            double turretY = pose.getY()
                    - ShooterConstants.kTurretOffsetMeters * Math.sin(pose.getRotation().getRadians());

            double hubX = isRed ? SuperstructureConstants.kRedHubX : SuperstructureConstants.kBlueHubX;
            double hubY = isRed ? SuperstructureConstants.kRedHubY : SuperstructureConstants.kBlueHubY;
            double dist = Math.hypot(hubX - turretX, hubY - turretY);

            // Flywheel + hood from distance interpolation
            m_shooter.setShooterStateFromDistance(dist, false);
            SmartDashboard.putNumber("Turret/DistToHubM", dist);

            // Turret: hub auto-aim if enabled, otherwise brake-hold current position
            m_turretAutoAim = SmartDashboard.getBoolean("Turret/AutoAim", m_turretAutoAim);
            if (m_turretAutoAim) {
                Rotation2d fieldAngle = new Rotation2d(Math.atan2(hubY - turretY, hubX - turretX));
                Rotation2d turretAngle = fieldAngle
                        .minus(pose.getRotation())
                        .minus(Rotation2d.fromDegrees(180.0));
                m_shooter.setTurretTargetDeg(turretAngle.getDegrees() + 180.0);
            } else {
                m_shooter.runTurretManual(0);
            }

            m_intake.runRollers();
            // Hopper only fires when turret + flywheels + hood are all on target
            if (m_shooter.isReadyToFire()) {
                m_hopper.startFeedSequence();
            } else {
                m_hopper.stop();
            }
        }, m_shooter, m_hopper)
        .finallyDo(() -> {
            m_shooter.stopFlywheels();
            m_hopper.stop();
            m_intake.stopRollers();
        })
    );

    // ==========================================================
    // SHOOTER DEFAULT COMMAND
    // Runs when no driver game command is holding RT/RB/LT.
    // Operator: A = flywheels, right stick Y = hood, right stick X = turret.
    // ==========================================================
    m_shooter.setDefaultCommand(Commands.run(() -> {
        // Keep SmartDashboard toggle and field in sync
        m_turretAutoAim = SmartDashboard.getBoolean("Turret/AutoAim", m_turretAutoAim);
        SmartDashboard.putBoolean("Turret/AutoAim", m_turretAutoAim);

        // Flywheels: hold A = m_flywheelTarget RPS, otherwise stop
        if (m_operatorController.a().getAsBoolean()) {
            m_shooter.setFlywheelVelocity(m_flywheelTarget);
        } else {
            m_shooter.stopFlywheels();
        }

        // Hood: right stick Y → open-loop nudge; only moves when stick active
        double hoodInput = MathUtil.applyDeadband(-m_operatorController.getRightY(), 0.1);
        if (hoodInput != 0.0) {
            m_shooter.setHoodPercent(hoodInput * -0.15);
        }

        // Turret: right stick X overrides to manual; then auto-aim; else brake-hold
        double turretInput = MathUtil.applyDeadband(m_operatorController.getRightX(), 0.1);
        if (turretInput != 0.0) {
            m_turretAutoAim = false;
            SmartDashboard.putBoolean("Turret/AutoAim", false);
            m_shooter.runTurretManual(turretInput);
        } else if (m_turretAutoAim) {
            var pose = m_swerve.getPose();
            boolean isRed = DriverStation.getAlliance()
                    .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
            double turretX = pose.getX()
                    - ShooterConstants.kTurretOffsetMeters * Math.cos(pose.getRotation().getRadians());
            double turretY = pose.getY()
                    - ShooterConstants.kTurretOffsetMeters * Math.sin(pose.getRotation().getRadians());
            double hubX = isRed ? SuperstructureConstants.kRedHubX : SuperstructureConstants.kBlueHubX;
            double hubY = isRed ? SuperstructureConstants.kRedHubY : SuperstructureConstants.kBlueHubY;
            SmartDashboard.putNumber("Turret/DistToHubM", Math.hypot(hubX - turretX, hubY - turretY));
            Rotation2d fieldAngle = new Rotation2d(Math.atan2(hubY - turretY, hubX - turretX));
            Rotation2d turretAngle = fieldAngle
                    .minus(pose.getRotation())
                    .minus(Rotation2d.fromDegrees(180.0));
            m_shooter.setTurretTargetDeg(turretAngle.getDegrees() + 180.0);
        } else {
            m_shooter.runTurretManual(0);
        }
    }, m_shooter));

    // ==========================================================
    // HOPPER DEFAULT COMMAND
    // B (hold): reverse for unjamming; otherwise stop.
    // ==========================================================
    m_hopper.setDefaultCommand(Commands.run(() -> {
        if (m_operatorController.b().getAsBoolean()) {
            m_hopper.reverse();
        } else {
            m_hopper.stop();
        }
    }, m_hopper));

    // ==========================================================
    // INTAKE DEFAULT COMMAND (ARM ONLY — rollers handled by driver)
    // Operator left stick Y → manual pivot override (sets MANUAL mode).
    // ZERO   : PID holds 0 rotations
    // JIGGLE : oscillates 0 ↔ kArmJiggleRotations on kArmJigglePeriodSec cycle
    // MANUAL : open-loop; releases when stick returns to center
    // ==========================================================
    m_intake.setDefaultCommand(Commands.run(() -> {
        double pivotInput = MathUtil.applyDeadband(m_operatorController.getLeftY(), 0.1);
        if (pivotInput != 0.0) {
            m_armMode = ArmMode.MANUAL;
            m_intake.runPivotManual(pivotInput);
        } else if (m_armMode == ArmMode.MANUAL) {
            m_intake.runPivotManual(0); // release; brake mode holds position
        } else if (m_armMode == ArmMode.JIGGLE) {
            double elapsed = m_jiggleTimer.get() % IntakeConstants.kArmJigglePeriodSec;
            double target = elapsed < IntakeConstants.kArmJigglePeriodSec / 2
                    ? IntakeConstants.kArmJiggleRotations : 0.0;
            m_intake.setPivotPosition(target);
        } else { // ZERO
            m_intake.setPivotPosition(0.0);
        }
    }, m_intake));

    // ==========================================================
    // OPERATOR BUTTONS — TESTING / OVERRIDE
    // ==========================================================

    // Left stick button: toggle turret auto-aim on/off
    m_operatorController.leftStick().onTrue(Commands.runOnce(() -> {
        m_turretAutoAim = !m_turretAutoAim;
        SmartDashboard.putBoolean("Turret/AutoAim", m_turretAutoAim);
    }));

    // RB: Hood homing — drive to hard stop, then zero encoder
    m_operatorController.rightBumper().onTrue(
        Commands.run(() -> m_shooter.setHoodPercent(0.20), m_shooter)
            .until(() -> m_shooter.getHoodCurrent() >
                SmartDashboard.getNumber("Hood/HomingCurrentThreshold", 15.0))
            .finallyDo(() -> {
                m_shooter.setHoodPercent(0);
                m_shooter.zeroHoodEncoder();
            })
    );

    // LB: Zero arm encoder at current position (use when arm is physically at stowed position)
    m_operatorController.leftBumper().onTrue(
        Commands.runOnce(() -> m_intake.zeroPivotEncoder())
    );

    // Y (hold): Run hopper + flywheels + intake together slowly (feed-sequence test)
    m_operatorController.y().whileTrue(
        Commands.run(() -> {
            m_shooter.setFlywheelVelocity(-30.0);
            m_hopper.runSlow();
            m_intake.runRollersSlow();
        }, m_shooter, m_hopper, m_intake)
        .finallyDo(() -> {
            m_shooter.stopFlywheels();
            m_hopper.stop();
            m_intake.stopRollers();
        })
    );

    // X (hold): Intake rollers slow for testing (no arm requirement — runs concurrently)
    m_operatorController.x().whileTrue(
        Commands.startEnd(
            () -> m_intake.runRollersSlow(),
            () -> m_intake.stopRollers()
        )
    );

    // RT: Snap hood to 2.0 rotations (testing preset)
    m_operatorController.rightTrigger().onTrue(
        Commands.runOnce(() -> {
            m_hoodTarget = 2.0;
            m_shooter.setHoodPosition(m_hoodTarget);
        })
    );

    // ==========================================================
    // D-PAD — CONTEXT-SENSITIVE SETPOINT TUNING
    //   RB held (passing active) → tunes pass setpoints
    //   Otherwise               → tunes shooter/dumb-vacuum setpoints
    // ==========================================================

    // Up/Down: ±1 RPS flywheel
    m_operatorController.povUp().onTrue(Commands.runOnce(() -> {
        if (m_passingActive) {
            m_passFlywheelRps += 1.0;
            SmartDashboard.putNumber("Pass/FlywheelRps", m_passFlywheelRps);
        } else {
            m_flywheelTarget -= 1.0; // more negative = faster
            SmartDashboard.putNumber("Shooter/SpeedSetpoint", m_flywheelTarget);
        }
    }));
    m_operatorController.povDown().onTrue(Commands.runOnce(() -> {
        if (m_passingActive) {
            m_passFlywheelRps -= 1.0;
            SmartDashboard.putNumber("Pass/FlywheelRps", m_passFlywheelRps);
        } else {
            m_flywheelTarget += 1.0; // less negative = slower
            SmartDashboard.putNumber("Shooter/SpeedSetpoint", m_flywheelTarget);
        }
    }));

    // Right/Left: ±0.2 rotations hood
    m_operatorController.povRight().onTrue(Commands.runOnce(() -> {
        if (m_passingActive) {
            m_passHoodRotations += 0.2;
            SmartDashboard.putNumber("Pass/HoodRotations", m_passHoodRotations);
        } else {
            m_hoodTarget += 0.2;
            m_shooter.setHoodPosition(m_hoodTarget);
        }
    }));
    m_operatorController.povLeft().onTrue(Commands.runOnce(() -> {
        if (m_passingActive) {
            m_passHoodRotations -= 0.2;
            SmartDashboard.putNumber("Pass/HoodRotations", m_passHoodRotations);
        } else {
            m_hoodTarget -= 0.2;
            m_shooter.setHoodPosition(m_hoodTarget);
        }
    }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
