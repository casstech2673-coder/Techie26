package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
        CREEP,       // Flywheels + intake roller + hopper at ~5% for testing
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
    private double m_manualPivotInput = 0.0;
    
    public Superstructure(SwerveDrive swerve, Intake intake, Hopper hopper, Shooter shooter, GameManager gameManager) {
        this.m_swerve = swerve;
        this.m_intake = intake;
        this.m_hopper = hopper;
        this.m_shooter = shooter;
        this.m_gameManager = gameManager;
    }

    @Override
    public void periodic() {
        // Motor control is handled entirely by commands in RobotContainer.
        // This class is kept only as a utility/telemetry holder.
    }

    // =========================================================================
    // PUBLIC CONTROLS
    // =========================================================================

    public void setState(SystemState newState) {
        m_currentState = newState;
    }

    public void setManualPivotInput(double joystickY) {
        m_manualPivotInput = joystickY;
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
        Pose2d pose = m_swerve.getPose();
        boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        // ── Turret's actual field position ────────────────────────────────────────
        // The turret pivot sits 7.25 in behind the robot center.
        // "Behind" in robot-frame is the −X_robot direction, so we rotate that
        // offset into field coordinates using the robot's current heading.
        double turretFieldX = pose.getX()
                - ShooterConstants.kTurretOffsetMeters * Math.cos(pose.getRotation().getRadians());
        double turretFieldY = pose.getY()
                - ShooterConstants.kTurretOffsetMeters * Math.sin(pose.getRotation().getRadians());
        Translation2d turretPos = new Translation2d(turretFieldX, turretFieldY);

        if (!aimAtHub) {
            // ── PASSING MODE ─────────────────────────────────────────────────────
            // Use a fixed global compass bearing so the turret locks to the
            // alliance wall no matter where the robot drives or how it's oriented.
            // Blue → 180° (X = 0 wall)   Red → 0° (X = 16.54 wall)
            double passGlobalBearingDeg = isRed ? 0.0 : 180.0;
            Rotation2d turretPassAngle = Rotation2d.fromDegrees(passGlobalBearingDeg)
                    .minus(pose.getRotation())       // field → robot-relative
                    .minus(Rotation2d.fromDegrees(180.0)); // subtract home offset
            // Convert from encoder-space angle to physical degrees (home offset = 180°)
            m_shooter.setTurretTargetDeg(turretPassAngle.getDegrees() + 180.0);

            // Distance from the turret (not robot center) to the wall for interpolation
            double wallX = isRed ? SuperstructureConstants.kRedAllianceWallX
                                 : SuperstructureConstants.kBlueAllianceWallX;
            double distToWall = Math.abs(wallX - turretFieldX);
            m_shooter.setShooterStateFromDistance(distToWall, true);
            return;
        }

        // ── HUB AIMING ───────────────────────────────────────────────────────────
        // All geometry is done from the turret's real position on the field, not
        // the robot center. This matters at close range where 7 in makes a
        // noticeable angular difference.
        Translation2d target = getTargetTranslation(true);

        // Distance from turret pivot to hub center → used for flywheel/hood lookup
        double dist = turretPos.getDistance(target);
        m_shooter.setShooterStateFromDistance(dist, false);

        // Field-relative angle from the turret to the hub
        double deltaX = target.getX() - turretFieldX;
        double deltaY = target.getY() - turretFieldY;
        Rotation2d fieldRelativeAngle = new Rotation2d(Math.atan2(deltaY, deltaX));

        // Convert to turret encoder space:
        //   − subtract robot heading  → robot-relative angle
        //   − subtract 180°           → turret-relative (encoder 0 = backward = 180°)
        Rotation2d turretAngle = fieldRelativeAngle
                .minus(pose.getRotation())
                .minus(Rotation2d.fromDegrees(180.0));
        // Convert from encoder-space angle to physical degrees (home offset = 180°)
        m_shooter.setTurretTargetDeg(turretAngle.getDegrees() + 180.0);
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

        // Compute from turret center, not robot center
        Pose2d pose = m_swerve.getPose();
        double heading = pose.getRotation().getRadians();
        Translation2d turret = new Translation2d(
            pose.getX() - ShooterConstants.kTurretOffsetMeters * Math.cos(heading),
            pose.getY() - ShooterConstants.kTurretOffsetMeters * Math.sin(heading)
        );
        return turret.getDistance(hub);
    }

}