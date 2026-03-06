package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

public class RobotTelemetry extends SubsystemBase {

    private final SwerveDrive m_swerve;
    private final Shooter m_shooter;
    private final Intake m_intake;
    private final Hopper m_hopper;
    private final Superstructure m_superStructure;
    private final GameManager m_gameManager;

    // MASTER DEBUG TOGGLE
    // Set to 'false' during real matches to prevent lag
    private final boolean kDebugMode = true;

    public RobotTelemetry(SwerveDrive swerve, Shooter shooter, Intake intake, 
                     Hopper hopper, Superstructure superstructure, GameManager gm) {
        this.m_swerve = swerve;
        this.m_shooter = shooter;
        this.m_intake = intake;
        this.m_hopper = hopper;
        this.m_superStructure = superstructure;
        this.m_gameManager = gm;
    }

    @Override
    public void periodic() {
        // ==========================================================
        // DRIVER CRITICAL DATA (Always On)
        // ==========================================================
        SmartDashboard.putString("DRIVER/System State", m_superStructure.getState().toString());
        SmartDashboard.putBoolean("DRIVER/Shooter Ready", m_shooter.isReadyToFire());
        
        // Pulls string from game manager
        SmartDashboard.putString("DRIVER/Hub Status", m_gameManager.getHubStatusString());

        // ==========================================================
        // MECHANISM DATA (Always On)
        // ==========================================================
        SmartDashboard.putNumber("MECH/Match Time Left", Math.round(DriverStation.getMatchTime()));
        SmartDashboard.putNumber("MECH/Shooter RPS", m_shooter.getFlywheelRps());
        SmartDashboard.putNumber("MECH/Intake Angle", m_intake.getPivotAngle());
        SmartDashboard.putString("MECH/Target Mode", m_superStructure.getTargetMode().toString());

        // ==========================================================
        // DEBUG & TUNING DATA (Toggleable)
        // ==========================================================
        if (kDebugMode) {
            // Swerve
            var pose = m_swerve.getPose();
            SmartDashboard.putNumber("DEBUG/Pose X", pose.getX());
            SmartDashboard.putNumber("DEBUG/Pose Y", pose.getY());
            SmartDashboard.putNumber("DEBUG/Pose Heading", pose.getRotation().getDegrees());

            // Shooter
            SmartDashboard.putNumber("DEBUG/Turret Rotations", m_shooter.getTurretRotations());
            SmartDashboard.putNumber("DEBUG/Hood Rotations", m_shooter.getHoodRotations());

            // Motor Outputs & Currents
            SmartDashboard.putNumber("DEBUG/Intake Output", m_intake.getPivotOutput());
            SmartDashboard.putNumber("DEBUG/Hopper Current", m_hopper.getVortexCurrent());
            SmartDashboard.putBoolean("DEBUG/Hopper Feeding", m_hopper.isFeeding());
        }

        // ==========================================================
        // TUNING DATA (For Calibration)
        // ==========================================================
        if (kDebugMode) {
            // Distance calculations for your Interpolation Maps
            // These pull the math directly from Superstructure so you know they match!
            SmartDashboard.putNumber("TUNING/Dist to Hub (m)", m_superStructure.getDistToHub());
            SmartDashboard.putNumber("TUNING/Dist to Wall (m)", m_superStructure.getDistToWall());

            // Target Values (What the robot is CURRENTLY trying to hit)
            // This helps you see if the interpolation is working correctly.
            SmartDashboard.putNumber("TUNING/Map Target RPS", m_shooter.getTargetRps());
            SmartDashboard.putNumber("TUNING/Map Target Hood", m_shooter.getTargetHood());
            
            // The TUNING/Live Test RPS/Hood will automatically appear next to these 
            // when you hold the Back button and run the TuneShooterCommand
        }
    }
}