package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Superstructure.SystemState;
import frc.robot.Superstructure.TargetMode;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TuneShooter;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  
  // ==========================================================
  // SUBSYSTEMS
  // ==========================================================
  private final SwerveDrive m_swerve = new SwerveDrive();
  private final Vision m_vision = new Vision(m_swerve);
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();
  private final Shooter m_shooter = new Shooter();

  // ==========================================================
  // CONTROLLERS
  // ==========================================================
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // ==========================================================
  // GAME MANAGEMENT & Telemetry
  // ==========================================================
  // The Game Manager handles match timers and rumbles the controllers
  private final GameManager m_gameManager = new GameManager(m_driverController, m_operatorController);

  // ==========================================================
  // SUPER STRUCTURE STATE MACHINE
  // ==========================================================
  private final Superstructure m_superstructure = new Superstructure(
      m_swerve, m_intake, m_hopper, m_shooter, m_gameManager);

  // ==========================================================
  // Telemetry
  // ==========================================================
  private final RobotTelemetry m_telemetry = new RobotTelemetry(m_swerve, m_shooter, m_intake, m_hopper, m_superstructure, m_gameManager);

  // Autonomous Chooser
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
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
    // PATHPLANNER AUTONOMOUS COMMANDS
    // ==========================================================

    // INTAKE: Deploys intake, runs rollers, stops balls in the hopper
    NamedCommands.registerCommand("Intake", 
        Commands.runOnce(() -> m_superstructure.setState(SystemState.COLLECTING))
    );
    
    // STOW: Tucks everything in for safe, high-speed driving
    NamedCommands.registerCommand("Stow", 
        Commands.runOnce(() -> m_superstructure.setState(SystemState.STOWED))
    );
    
    // SCORE: Fires the ball, waits for it to shoot
    NamedCommands.registerCommand("Score", 
        Commands.runOnce(() -> m_superstructure.setState(SystemState.FIRING))
            // Wait for the hopper's run-up sequence and the physical shot to clear
            .andThen(Commands.waitSeconds(0.75)) 
            // Shut the shooter back down so we aren't wasting battery
            .andThen(Commands.runOnce(() -> m_superstructure.setState(SystemState.IDLE)))
    );

    // VACUUM: Drive over a piece and instantly shoot it without stopping
    NamedCommands.registerCommand("Vacuum", 
        Commands.runOnce(() -> m_superstructure.setState(SystemState.VACUUM))
    );

    // PathPlanner Auto Chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Select Auto", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // ==========================================================
    // DRIVER BUTTONS
    // ==========================================================
    // Y: Set robot 0 degrees (forward)
    // B: Set robot -90 degrees (right)
    // A: Set robot 180 degrees (toward driver)
    // X: Set robot 90 degrees (left)
    
    // Zero the Gyro (Start Button)
    m_driverController.start().onTrue(Commands.runOnce(() -> m_swerve.zeroHeading()));

    // Set X-Stance (Hold Left Bumper)
    m_driverController.leftBumper().whileTrue(Commands.runOnce(() -> m_swerve.setX()));


    // ==========================================================
    // OPERATOR BUTTONS
    // ==========================================================

    // VACUUM (Hold Left Trigger)
    // Floor-to-shooter auto-targeting. When let go, it returns to IDLE.
    m_operatorController.leftTrigger()
        .onTrue(Commands.runOnce(() -> m_superstructure.setState(SystemState.VACUUM)))
        .onFalse(Commands.runOnce(() -> m_superstructure.setState(SystemState.IDLE)));

    // FIRE (Hold Right Trigger)
    // Triggers the Hopper feed sequence (run-up -> fire). 
    // Letting go drops it directly into COLLECTING so trailing balls are saved!
    m_operatorController.rightTrigger()
        .onTrue(Commands.runOnce(() -> m_superstructure.setState(SystemState.FIRING)))
        .onFalse(Commands.runOnce(() -> m_superstructure.setState(SystemState.COLLECTING)));

    // COLLECT (Hold Left Bumper)
    // Intakes balls but stops them in the hopper. Shooter stays warm.
    m_operatorController.leftBumper()
        .onTrue(Commands.runOnce(() -> m_superstructure.setState(SystemState.COLLECTING)))
        .onFalse(Commands.runOnce(() -> m_superstructure.setState(SystemState.IDLE)));

    // EMERGENCY STOW (Press B Button)
    // Instantly aborts everything, pulls intake in, and shuts off motors.
    m_operatorController.b()
        .onTrue(Commands.runOnce(() -> m_superstructure.setState(SystemState.STOWED)));

    // STOP / IDLE (Press X Button)
    // A dedicated button to stop collecting and just idle if you don't want to stow.
    m_operatorController.x()
        .onTrue(Commands.runOnce(() -> m_superstructure.setState(SystemState.IDLE)));

    // A BUTTON: HYBRID Target TOGGLE
    m_operatorController.a()
        // Tap to cycle between Hub and Wall
        .onTrue(Commands.runOnce(() -> m_superstructure.cycleManualTargets()))
        
        // IF HELD FOR 1 SECOND: Reset back to AUTO
        .onTrue(
            Commands.waitSeconds(1.0)
                .andThen(Commands.runOnce(() -> {
                    if (m_operatorController.getHID().getAButton()) {
                        m_superstructure.setTargetMode(TargetMode.AUTO);
                    }
                }
            )
        ));

    // LIVE TUNING MODE (Hold Back Button)
    // Engages the D-Pad tuning command we built earlier so you can calibrate the interpolation maps.
    m_operatorController.back()
        .whileTrue(new TuneShooter(m_shooter, m_operatorController));
  }

  /**
   * Passes the autonomous command selected on the dashboard to the main Robot class.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}