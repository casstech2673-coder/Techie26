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
        SmartDashboard.putString("DRIVER/Hub Status", m_gameManager.getHubStatusString());

        // ==========================================================
        // MATCH (Always On)
        // ==========================================================
        SmartDashboard.putNumber("MECH/Match Time Left", Math.round(DriverStation.getMatchTime()));
        SmartDashboard.putString("MECH/Target Mode", m_superStructure.getTargetMode().toString());

        // ==========================================================
        // SHOOTER (Always On)
        // ==========================================================
        SmartDashboard.putNumber("Shooter/FlywheelRPS",    m_shooter.getFlywheelRps());
        SmartDashboard.putNumber("Shooter/TargetRPS",      m_shooter.getTargetRps());
        SmartDashboard.putNumber("Hood/Rotations",         m_shooter.getHoodRotations());
        SmartDashboard.putNumber("Hood/TargetRotations",   m_shooter.getTargetHood());
        SmartDashboard.putNumber("Hood/CurrentAmps",       m_shooter.getHoodCurrent());
        SmartDashboard.putNumber("Turret/Rotations",       m_shooter.getTurretRotations());
        SmartDashboard.putNumber("Turret/AngleDeg",        m_shooter.getTurretAngleDeg());
        SmartDashboard.putNumber("Turret/TargetDeg",       m_shooter.getTargetTurretDeg());
        SmartDashboard.putNumber("Turret/ErrorDeg",        m_shooter.getTargetTurretDeg() - m_shooter.getTurretAngleDeg());

        // ==========================================================
        // INTAKE & HOPPER (Always On)
        // ==========================================================
        SmartDashboard.putNumber("Intake/ArmRotations",   m_intake.getPivotPosition());
        SmartDashboard.putNumber("Intake/PivotOutput",    m_intake.getPivotOutput());
        SmartDashboard.putNumber("Hopper/CurrentAmps",    m_hopper.getVortexCurrent());
        SmartDashboard.putBoolean("Hopper/IsFeeding",     m_hopper.isFeeding());

        // ==========================================================
        // DEBUG & TUNING (kDebugMode only — set false during matches)
        // ==========================================================
        if (kDebugMode) {
            var pose = m_swerve.getPose();
            SmartDashboard.putNumber("DEBUG/Pose X",       pose.getX());
            SmartDashboard.putNumber("DEBUG/Pose Y",       pose.getY());
            SmartDashboard.putNumber("DEBUG/Pose Heading", pose.getRotation().getDegrees());

            // Distance math from Superstructure — confirms interpolation inputs are correct
            SmartDashboard.putNumber("TUNING/Dist to Hub (m)",  m_superStructure.getDistToHub());
            SmartDashboard.putNumber("TUNING/Dist to Wall (m)", m_superStructure.getDistToWall());
            SmartDashboard.putNumber("TUNING/Map Target RPS",   m_shooter.getTargetRps());
            SmartDashboard.putNumber("TUNING/Map Target Hood",  m_shooter.getTargetHood());
            // TUNING/Current Test RPS and TUNING/Current Test Hood appear here while TuneShooter runs
        }
    }
}