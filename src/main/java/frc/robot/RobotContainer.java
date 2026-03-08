package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands; // TESTING MODE: commented out

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.Superstructure.SystemState; // TESTING MODE: commented out
// import frc.robot.Superstructure.TargetMode;  // TESTING MODE: commented out
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TeleopDrive;
// import frc.robot.commands.TuneShooter; // TESTING MODE: commented out
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
// import frc.robot.subsystems.Vision; // TESTING MODE: commented out

public class RobotContainer {

  // ==========================================================
  // SUBSYSTEMS
  // ==========================================================
  private final SwerveDrive m_swerve = new SwerveDrive();
  // private final Vision m_vision = new Vision(m_swerve); // TESTING MODE: commented out (used by Superstructure only)
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();
  private final Shooter m_shooter = new Shooter();

  // ==========================================================
  // CONTROLLERS
  // ==========================================================
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // ==========================================================
  // GAME MANAGEMENT — TESTING MODE: commented out
  // ==========================================================
  // private final GameManager m_gameManager = new GameManager(m_driverController, m_operatorController);

  // ==========================================================
  // SUPER STRUCTURE STATE MACHINE — TESTING MODE: commented out
  // ==========================================================
  // private final Superstructure m_superstructure = new Superstructure(
  //     m_swerve, m_vision, m_intake, m_hopper, m_shooter, m_gameManager);

  // ==========================================================
  // Telemetry — TESTING MODE: commented out (requires Superstructure)
  // ==========================================================
  // private final RobotTelemetry m_telemetry = new RobotTelemetry(m_swerve, m_shooter, m_intake, m_hopper, m_superstructure, m_gameManager);

  // Autonomous Chooser
  private final SendableChooser<Command> autoChooser;

  // Testing mode: adjustable flywheel/hood setpoints
  private double m_flywheelTarget = -10.0; // RPS (negative = forward)
  private double m_hoodTarget = 0.0;       // encoder rotations

  public RobotContainer() {
    // Drivetrain default command — unchanged
    m_swerve.setDefaultCommand(
        new TeleopDrive(
            m_swerve,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> true,
            () -> m_driverController.a().getAsBoolean(), // A Button
            () -> m_driverController.b().getAsBoolean(), // B Button
            () -> m_driverController.x().getAsBoolean(), // X Button
            () -> m_driverController.y().getAsBoolean()  // Y Button
        )
    );

    // ==========================================================
    // PATHPLANNER NAMED COMMANDS — TESTING MODE: commented out
    // ==========================================================
    // NamedCommands.registerCommand("Intake",
    //     Commands.runOnce(() -> m_superstructure.setState(SystemState.COLLECTING))
    // );
    // NamedCommands.registerCommand("Stow",
    //     Commands.runOnce(() -> m_superstructure.setState(SystemState.STOWED))
    // );
    // NamedCommands.registerCommand("Score",
    //     Commands.runOnce(() -> m_superstructure.setState(SystemState.FIRING))
    //         .andThen(Commands.waitSeconds(0.75))
    //         .andThen(Commands.runOnce(() -> m_superstructure.setState(SystemState.IDLE)))
    // );
    // NamedCommands.registerCommand("Vacuum",
    //     Commands.runOnce(() -> m_superstructure.setState(SystemState.VACUUM))
    // );

    if (AutoBuilder.isConfigured()) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = new SendableChooser<>();
      DriverStation.reportWarning("AutoBuilder not configured — auto routines unavailable. "
          + "Open PathPlanner GUI, configure the robot, and redeploy.", false);
    }
    SmartDashboard.putData("Select Auto", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // ==========================================================
    // DRIVER BUTTONS — unchanged
    // ==========================================================

    // Zero the Gyro (Start Button)
    m_driverController.start().onTrue(Commands.runOnce(() -> m_swerve.zeroHeading()));

    // Set X-Stance (Hold Left Bumper)
    m_driverController.leftBumper().whileTrue(Commands.run(() -> m_swerve.setX()));


    // ==========================================================
    // *** TESTING MODE — OPERATOR CONTROLLER ***
    //
    // A (hold) : Flywheels spin at very slow speed; release = stop
    // B (hold) : Hopper runs very slowly; release = stop
    // X (hold) : Intake rollers run; release = stop
    // Right stick Y : Hood up/down slowly; center = stop
    // Left stick Y  : Intake pivot up/down slowly; center = stop
    // ==========================================================

    // --- Shooter default command ---
    // Reads A for flywheels and right stick Y for hood every tick.
    // Hood stops (holds with 0 output) when stick is centered.
    m_shooter.setDefaultCommand(Commands.run(() -> {
        // Flywheels: hold A = m_flywheelTarget RPS (tunable via D-pad), otherwise stop
        if (m_operatorController.a().getAsBoolean()) {
            m_shooter.setFlywheelVelocity(m_flywheelTarget);
        } else {
            m_shooter.stopFlywheels();
        }

        // Hood: right stick Y → open-loop slow; only runs when stick is active
        // (leaves position control intact when stick is centered)
        double hoodInput = MathUtil.applyDeadband(-m_operatorController.getRightY(), 0.1);
        if (hoodInput != 0.0) {
            m_shooter.setHoodPercent(hoodInput * -1.0);
        }
    }, m_shooter));

    // --- Hopper default command ---
    // Hold B = run slow; release = stop.
    m_hopper.setDefaultCommand(Commands.run(() -> {
        if (m_operatorController.b().getAsBoolean()) {
            m_hopper.reverse();
        } else {
            m_hopper.stop();
        }
    }, m_hopper));

    // RB : Zero hood encoder at current position
    
    // RB: Home hood — drive backwards until amp spike, then zero encoder
    m_operatorController.rightBumper().onTrue(
        Commands.run(() -> m_shooter.setHoodPercent(0.10), m_shooter)
            .until(() -> m_shooter.getHoodCurrent() >
                SmartDashboard.getNumber("Hood/HomingCurrentThreshold", 8.0))
            .finallyDo(() -> {
                m_shooter.setHoodPercent(0);
                m_shooter.zeroHoodEncoder();
            })
    );

    // LB : Zero arm (intake pivot) encoder at current position
    m_operatorController.leftBumper().onTrue(
        Commands.runOnce(() -> m_intake.zeroPivotEncoder())
    );

    // Y (hold) : Hopper + flywheels + intake rollers run slowly together; release = stop all
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

    // D-pad up/down: adjust flywheel speed setpoint ±1 RPS
    m_operatorController.povUp().onTrue(Commands.runOnce(() -> {
        m_flywheelTarget -= 1.0; // more negative = faster
        SmartDashboard.putNumber("Shooter/SpeedSetpoint", m_flywheelTarget);
    }));
    m_operatorController.povDown().onTrue(Commands.runOnce(() -> {
        m_flywheelTarget += 1.0; // less negative = slower
        SmartDashboard.putNumber("Shooter/SpeedSetpoint", m_flywheelTarget);
    }));

    // Right Trigger: set hood to 20 degrees (converted via 36:1 gear ratio)
    m_operatorController.rightTrigger().onTrue(
        Commands.runOnce(() -> {
            m_hoodTarget = 20.0 * ShooterConstants.kHoodDegreesToRotations;
            m_shooter.setHoodPosition(m_hoodTarget);
        }, m_shooter)
    );

    // D-pad right/left: adjust hood position ±2 degrees
    m_operatorController.povRight().onTrue(
        Commands.runOnce(() -> {
            m_hoodTarget += 2.0 * ShooterConstants.kHoodDegreesToRotations;
            m_shooter.setHoodPosition(m_hoodTarget);
        }, m_shooter)
    );
    m_operatorController.povLeft().onTrue(
        Commands.runOnce(() -> {
            m_hoodTarget -= 2.0 * ShooterConstants.kHoodDegreesToRotations;
            m_shooter.setHoodPosition(m_hoodTarget);
        }, m_shooter)
    );

    // --- Intake default command ---
    // Hold X = rollers slow; left stick Y = pivot manual (zeroes at center).
    m_intake.setDefaultCommand(Commands.run(() -> {
        // Rollers: hold X = slow, otherwise stop
        if (m_operatorController.x().getAsBoolean()) {
            m_intake.runRollersSlow();
        } else {
            m_intake.stopRollers();
        }

        // Pivot: left stick Y → manual open-loop; runPivotManual(0) = motor off
        double pivotInput = MathUtil.applyDeadband(-m_operatorController.getLeftY(), 0.1);
        m_intake.runPivotManual(pivotInput);
    }, m_intake));


    // ==========================================================
    // OPERATOR BUTTONS — TESTING MODE: commented out
    // ==========================================================

    // VACUUM (Hold Left Trigger)
    // m_operatorController.leftTrigger()
    //     .onTrue(Commands.runOnce(() -> m_superstructure.setState(SystemState.VACUUM)))
    //     .onFalse(Commands.runOnce(() -> m_superstructure.setState(SystemState.IDLE)));

    // FIRE (Hold Right Trigger)
    // m_operatorController.rightTrigger()
    //     .onTrue(Commands.runOnce(() -> m_superstructure.setState(SystemState.FIRING)))
    //     .onFalse(Commands.runOnce(() -> m_superstructure.setState(SystemState.COLLECTING)));

    // COLLECT (Hold Left Bumper)
    // m_operatorController.leftBumper()
    //     .onTrue(Commands.runOnce(() -> m_superstructure.setState(SystemState.COLLECTING)))
    //     .onFalse(Commands.runOnce(() -> m_superstructure.setState(SystemState.IDLE)));

    // EMERGENCY STOW (Press B Button)
    // m_operatorController.b()
    //     .onTrue(Commands.runOnce(() -> m_superstructure.setState(SystemState.STOWED)));

    // STOP / IDLE (Press X Button)
    // m_operatorController.x()
    //     .onTrue(Commands.runOnce(() -> m_superstructure.setState(SystemState.IDLE)));

    // A BUTTON: HYBRID Target TOGGLE
    // m_operatorController.a()
    //     .onTrue(Commands.runOnce(() -> m_superstructure.cycleManualTargets()));

    // m_operatorController.a().debounce(1.0)
    //     .onTrue(Commands.runOnce(() -> m_superstructure.setTargetMode(TargetMode.AUTO)));

    // CREEP (Hold Right Bumper)
    // m_operatorController.rightBumper()
    //     .whileTrue(Commands.run(() -> {
    //         m_superstructure.setState(SystemState.CREEP);
    //         m_superstructure.setManualPivotInput(-m_operatorController.getLeftY());
    //     }))
    //     .onFalse(Commands.runOnce(() -> {
    //         m_superstructure.setManualPivotInput(0.0);
    //         m_superstructure.setState(SystemState.IDLE);
    //     }));

    // LIVE TUNING MODE (Hold Back Button)
    // m_operatorController.back()
    //     .whileTrue(new TuneShooter(m_shooter, m_operatorController));
  }

  /**
   * Passes the autonomous command selected on the dashboard to the main Robot class.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
