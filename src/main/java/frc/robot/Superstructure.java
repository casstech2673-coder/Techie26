package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

public class Superstructure extends SubsystemBase {

    public enum SystemState {
        STOWED,      // Everything tucked in safely (Defense mode)
        IDLE,        // Intake deployed, rollers off, shooter warm
        COLLECTING,  // Intake deployed, rollers spinning, hopper stopped
        VACUUM,      // Intake + Hopper + Shooter (Auto-Targeting)
        FIRING,      // Hopper slams forward into the shooter
    }

    public enum TargetMode { 
        AUTO, 
        FORCE_HUB, 
        FORCE_WALL 
    }

    // Subsystems
    private final SwerveDrive m_swerve;
    private final Intake m_intake;
    private final Hopper m_hopper;
    private final Shooter m_shooter;
    private final GameManager m_gameManager;

    // State Tracking
    private SystemState m_currentState = SystemState.STOWED;
    private TargetMode m_targetMode = TargetMode.AUTO;
    
    public Superstructure(SwerveDrive swerve, Intake intake, Hopper hopper, Shooter shooter, GameManager gameManager) {
        this.m_swerve = swerve;
        this.m_intake = intake;
        this.m_hopper = hopper;
        this.m_shooter = shooter;
        this.m_gameManager = gameManager;
    }

    @Override
    public void periodic() {
        switch (m_currentState) {
            case STOWED:
                m_intake.setPivotAngle(IntakeConstants.kStowedAngle);
                m_intake.stopRollers();
                m_hopper.stop();
                m_shooter.stopShooter();
                break;

            case IDLE:
                m_intake.setPivotAngle(IntakeConstants.kDeployedAngle);
                m_intake.stopRollers();
                m_hopper.stop();
                m_shooter.setFlywheelVelocity(10.0); // Keep shooter idle
                break;

            case COLLECTING:
                m_intake.setPivotAngle(IntakeConstants.kDeployedAngle);
                m_intake.runRollers();
                m_hopper.stop();
                m_shooter.setFlywheelVelocity(10.0);
                break;

            case VACUUM:
                m_intake.setPivotAngle(IntakeConstants.kDeployedAngle);
                m_intake.runRollers();
                m_hopper.startFeedSequence();
                handleShooterAiming(); 
                break;

            case FIRING:
                m_intake.setPivotAngle(IntakeConstants.kDeployedAngle);
                m_intake.runRollers();
                m_hopper.startFeedSequence(); // Slam it forward into flywheels
                handleShooterAiming();
                break;
        }
    }

    // =========================================================================
    // PUBLIC CONTROLS
    // =========================================================================

    public void setState(SystemState newState) {
        m_currentState = newState;
    }

    public void setTargetMode(TargetMode mode) {
        m_targetMode = mode;
    }

    public void cycleManualTargets() {
        if (m_targetMode == TargetMode.AUTO || m_targetMode == TargetMode.FORCE_WALL) {
            m_targetMode = TargetMode.FORCE_HUB;
        } else {
            m_targetMode = TargetMode.FORCE_WALL;
        }
    }

    // =========================================================================
    // INTERNAL LOGIC & DASHBOARD
    // =========================================================================

    private void handleShooterAiming() {
        boolean aimAtHub = isTargetingHub();
        Translation2d target = getTargetTranslation(aimAtHub);
        Pose2d pose = m_swerve.getPose();
        
        // Calculate Distance (For Hood & Flywheels)
        double dist = pose.getTranslation().getDistance(target);
        m_shooter.setShooterStateFromDistance(dist, !aimAtHub);
        
        // Calculate Angle (For the Turret)
        double deltaX = target.getX() - pose.getX();
        double deltaY = target.getY() - pose.getY();
        
        Rotation2d fieldRelativeAngle = new Rotation2d(Math.atan2(deltaY, deltaX));
        Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(pose.getRotation());
        
        // Convert to Motor Rotations
        double targetMotorRotations = robotRelativeAngle.getRotations() * ShooterConstants.kTurretMotorRotationsPerTurretRevolution;
        
        m_shooter.setTurretPosition(targetMotorRotations);
    }

    public SystemState getState() { return m_currentState; }
    public TargetMode getTargetMode() { return m_targetMode; }
    // Change your private isTargetingHub() to public so Telemetry can read it:
    public boolean isTargetingHub() {
        if (m_targetMode == TargetMode.FORCE_HUB) return true;
        if (m_targetMode == TargetMode.FORCE_WALL) return false;
        return m_gameManager.isHubActive();
    }

    private Translation2d getTargetTranslation(boolean isHub) {
        boolean isRed = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            isRed = true;
        }

        if (isHub) {
            // Use specific Hub coordinates based on alliance
            return new Translation2d(
                isRed ? SuperstructureConstants.kRedHubX : SuperstructureConstants.kBlueHubX,
                isRed ? SuperstructureConstants.kRedHubY : SuperstructureConstants.kBlueHubY
            );
        } else {
            // Target the Alliance Wall (Match Y with robot to pass straight back)
            double wallX = isRed ? SuperstructureConstants.kRedAllianceWallX : SuperstructureConstants.kBlueAllianceWallX;
            return new Translation2d(wallX, m_swerve.getPose().getY());
        }
    }

    /** * Returns distance to the Hub center. 
     * Used for the Score Interpolation Map.
     */
    public double getDistToHub() {
        boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        Translation2d hub = isRed ? 
            new Translation2d(SuperstructureConstants.kRedHubX, SuperstructureConstants.kRedHubY) : 
            new Translation2d(SuperstructureConstants.kBlueHubX, SuperstructureConstants.kBlueHubY);
        
        return m_swerve.getPose().getTranslation().getDistance(hub);
    }

    /** * Returns distance to the Alliance Wall. 
     * Used for the Pass/Ferry Interpolation Map.
     */
    public double getDistToWall() {
        boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        double wallX = isRed ? SuperstructureConstants.kRedAllianceWallX : SuperstructureConstants.kBlueAllianceWallX;
        return Math.abs(wallX - m_swerve.getPose().getX());
    }
}