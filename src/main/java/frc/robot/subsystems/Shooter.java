package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    // --- Turret: NEO Vortex on SparkFlex ---
    private final SparkFlex m_turretMotor = new SparkFlex(ShooterConstants.kTurretId, MotorType.kBrushless);
    private final RelativeEncoder m_turretEncoder = m_turretMotor.getEncoder();
    private final SparkClosedLoopController m_turretController = m_turretMotor.getClosedLoopController();

    // --- Hood, Flywheels: TalonFX ---
    private final TalonFX m_hoodMotor = new TalonFX(ShooterConstants.kHoodId);
    private final TalonFX m_leftFlywheel = new TalonFX(ShooterConstants.kLeftFlywheelId);
    private final TalonFX m_rightFlywheel = new TalonFX(ShooterConstants.kRightFlywheelId);

    // TalonFX Control Requests (hood + flywheels only)
    private final PositionVoltage m_positionRequest = new PositionVoltage(0);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

    // Interpolation Maps (Distance in Meters -> Encoder Space Values)
    private final InterpolatingDoubleTreeMap m_scoreRpsMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_scoreHoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_passRpsMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_passHoodMap = new InterpolatingDoubleTreeMap();

    // Target States for Tolerance Checking
    private double m_targetTurretRotations = 0.0;
    private double m_targetHoodRotations = 0.0;
    private double m_targetFlywheelRps = 0.0;

    public Shooter() {
        configureMotors();
        populateInterpolationMaps();
    }

    private void configureMotors() {
        // --- TURRET (SparkFlex / NEO Vortex, Position Control) ---
        SparkFlexConfig turretConfig = new SparkFlexConfig();
        turretConfig.idleMode(IdleMode.kBrake);
        turretConfig.smartCurrentLimit(40);
        turretConfig.closedLoop
                .pid(ShooterConstants.kTurretP, ShooterConstants.kTurretI, ShooterConstants.kTurretD)
                .outputRange(-0.5, 0.5);
        turretConfig.softLimit
                .forwardSoftLimit((float) ShooterConstants.kTurretMaxRotations)
                .reverseSoftLimit((float) ShooterConstants.kTurretMinRotations)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        m_turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- FLYWHEELS (Velocity Control) ---
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = ShooterConstants.kFlywheelP;
        flywheelConfig.Slot0.kI = ShooterConstants.kFlywheelI;
        flywheelConfig.Slot0.kD = ShooterConstants.kFlywheelD;
        flywheelConfig.Slot0.kV = ShooterConstants.kFlywheelV;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 80;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_leftFlywheel.getConfigurator().apply(flywheelConfig);
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_rightFlywheel.getConfigurator().apply(flywheelConfig);

        m_leftFlywheel.setNeutralMode(NeutralModeValue.Coast);
        m_rightFlywheel.setNeutralMode(NeutralModeValue.Coast);

        // --- HOOD (TalonFX Position Control) ---
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = ShooterConstants.kPositionP;
        hoodConfig.Slot0.kI = ShooterConstants.kPositionI;
        hoodConfig.Slot0.kD = ShooterConstants.kPositionD;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 40;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_hoodMotor.getConfigurator().apply(hoodConfig);
        m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private void populateInterpolationMaps() {
        // TODO: Scoring truth table (Values must be tuned for distance to RPS / Hood Motor Rotations)
        m_scoreRpsMap.put(1.5, 50.0);  m_scoreHoodMap.put(1.5, 2.5);
        m_scoreRpsMap.put(3.0, 75.0);  m_scoreHoodMap.put(3.0, 4.0);
        m_scoreRpsMap.put(5.0, 100.0); m_scoreHoodMap.put(5.0, 5.5);

        // Passing truth table
        m_passRpsMap.put(3.0, 35.0);  m_passHoodMap.put(3.0, 8.0);
        m_passRpsMap.put(7.0, 55.0);  m_passHoodMap.put(7.0, 7.0);
    }

    @Override
    public void periodic() {}

    // ==========================================================
    // COMMANDS
    // ==========================================================

    public void setShooterStateFromDistance(double distanceMeters, boolean isPassMode) {
        if (isPassMode) {
            setHoodPosition(m_passHoodMap.get(distanceMeters));
            setFlywheelVelocity(m_passRpsMap.get(distanceMeters));
        } else {
            setHoodPosition(m_scoreHoodMap.get(distanceMeters));
            setFlywheelVelocity(m_scoreRpsMap.get(distanceMeters));
        }
    }

    public void setFlywheelVelocity(double targetRps) {
        m_targetFlywheelRps = targetRps;
        m_leftFlywheel.setControl(m_velocityRequest.withVelocity(targetRps));
        m_rightFlywheel.setControl(m_velocityRequest.withVelocity(targetRps));
    }

    public void setHoodPosition(double targetRotations) {
        m_targetHoodRotations = targetRotations;
        m_hoodMotor.setControl(m_positionRequest.withPosition(targetRotations));
    }

    /**
     * Commands the turret to a target rotation via shortest-path, respecting soft limits.
     */
    public void setTurretPosition(double targetRotations) {
        double currentRotations = getTurretRotations();
        double fullSweep = ShooterConstants.kTurretMotorRotationsPerTurretRevolution;

        // Find shortest path
        double error = MathUtil.inputModulus(targetRotations - currentRotations, -fullSweep / 2.0, fullSweep / 2.0);
        double continuousTarget = currentRotations + error;

        // If shortest path breaks a wire, go the long way
        if (continuousTarget > ShooterConstants.kTurretMaxRotations) {
            continuousTarget -= fullSweep;
        } else if (continuousTarget < ShooterConstants.kTurretMinRotations) {
            continuousTarget += fullSweep;
        }

        continuousTarget = MathUtil.clamp(
            continuousTarget,
            ShooterConstants.kTurretMinRotations,
            ShooterConstants.kTurretMaxRotations
        );

        m_targetTurretRotations = continuousTarget;
        m_turretController.setReference(continuousTarget, ControlType.kPosition);
    }

    /** Stops only the flywheels (for test mode). */
    public void stopFlywheels() {
        m_leftFlywheel.stopMotor();
        m_rightFlywheel.stopMotor();
    }

    /** Open-loop hood control for test mode (-1 to 1). */
    public void setHoodPercent(double percent) {
        m_hoodMotor.setControl(m_dutyCycleRequest.withOutput(percent));
    }

    public void stopShooter() {
        m_leftFlywheel.stopMotor();
        m_rightFlywheel.stopMotor();
        setTurretPosition(0.0);
        setHoodPosition(0.0);
    }

    public double getTargetRps() { return m_targetFlywheelRps; }
    public double getTargetHood() { return m_targetHoodRotations; }

    // ==========================================================
    // "READY TO FIRE" LOGIC
    // ==========================================================

    public double getFlywheelRps() {
        return m_leftFlywheel.getVelocity().getValueAsDouble();
    }

    public double getTurretRotations() {
        return m_turretEncoder.getPosition();
    }

    public double getHoodRotations() {
        return m_hoodMotor.getPosition().getValueAsDouble();
    }

    public boolean isReadyToFire() {
        boolean flywheelsReady = Math.abs(getFlywheelRps() - m_targetFlywheelRps) <= ShooterConstants.kFlywheelToleranceRPS;
        boolean turretReady = Math.abs(getTurretRotations() - m_targetTurretRotations) <= ShooterConstants.kTurretToleranceRotations;
        boolean hoodReady = Math.abs(getHoodRotations() - m_targetHoodRotations) <= ShooterConstants.kHoodToleranceRotations;

        if (m_targetFlywheelRps == 0) return false;

        return flywheelsReady && turretReady && hoodReady;
    }
}
