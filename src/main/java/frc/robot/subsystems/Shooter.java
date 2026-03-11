package frc.robot.subsystems;

// CTRE Imports (For Hood & Flywheels)
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// REV Imports (For the Turret — open-loop only, no closed-loop controller needed)
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    // --- Turret: NEO Vortex on SparkFlex (open-loop, lookup-table driven) ---
    private final SparkFlex m_turretMotor = new SparkFlex(ShooterConstants.kTurretId, MotorType.kBrushless);
    private final RelativeEncoder m_turretEncoder = m_turretMotor.getEncoder();

    // --- Hood, Flywheels: TalonFX ---
    private final TalonFX m_hoodMotor = new TalonFX(ShooterConstants.kHoodId);
    private final TalonFX m_leftFlywheel = new TalonFX(ShooterConstants.kLeftFlywheelId);
    private final TalonFX m_rightFlywheel = new TalonFX(ShooterConstants.kRightFlywheelId);

    // TalonFX Control Requests (hood + flywheels only)
    private final PositionVoltage m_positionRequest = new PositionVoltage(0);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

    // Flywheel/hood interpolation maps (distance in meters → encoder values)
    private final InterpolatingDoubleTreeMap m_scoreRpsMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_scoreHoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_passRpsMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_passHoodMap = new InterpolatingDoubleTreeMap();

    // Turret power lookup table: |degree error| → duty-cycle magnitude [0..1]
    // Applied symmetrically: positive error → positive power, negative → negative.
    private final InterpolatingDoubleTreeMap m_turretPowerMap = new InterpolatingDoubleTreeMap();

    // Target states for tolerance checking
    private double m_targetTurretDeg = 180.0; // 180° = home (straight backward)
    private double m_targetHoodRotations = 0.0;
    private double m_targetFlywheelRps = 0.0;

    // Hood live-tuning state
    private double m_hoodMaxPercent = 0.20;
    private double m_lastHoodP = ShooterConstants.kPositionP;
    private double m_lastHoodI = ShooterConstants.kPositionI;
    private double m_lastHoodD = ShooterConstants.kPositionD;

    public Shooter() {
        configureMotors();
        populateInterpolationMaps();
        populateTurretPowerMap();

        SmartDashboard.putNumber("Hood/HomingCurrentThreshold", 8.0);
        SmartDashboard.putNumber("Hood/kP", ShooterConstants.kPositionP);
        SmartDashboard.putNumber("Hood/kI", ShooterConstants.kPositionI);
        SmartDashboard.putNumber("Hood/kD", ShooterConstants.kPositionD);
        SmartDashboard.putNumber("Hood/MaxPercent", 0.30);
    }

    private void configureMotors() {
        // --- TURRET (SparkFlex / NEO Vortex, open-loop duty cycle) ---
        // No closedLoop block needed — we drive it open-loop via m_turretMotor.set().
        // Brake mode makes it hold position passively when power = 0.
        // Soft limits are hardware-enforced by the SparkFlex even in open-loop.
        SparkFlexConfig turretConfig = new SparkFlexConfig();
        turretConfig.idleMode(IdleMode.kBrake);
        turretConfig.smartCurrentLimit(40);
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
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: verify on real robot
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

    /**
     * Turret power lookup: |error in degrees| → duty-cycle magnitude.
     *
     *  Error (°)  │  Power
     *  ───────────┼────────
     *     0       │  0.00   within tolerance — no correction
     *     2       │  0.03   tiny nudge
     *     5       │  0.07
     *    10       │  0.14
     *    20       │  0.25
     *    45       │  0.40
     *    90       │  0.50   max (matches the soft-limit range)
     *
     * TODO: Tune on the real robot. If the turret overshoots, reduce the upper
     * values. If it's too sluggish, increase them. Start with the robot on blocks.
     *
     * NOTE: If the turret moves away from the target instead of toward it, negate
     * the power in setTurretTargetDeg() — motor direction depends on wiring.
     */
    private void populateTurretPowerMap() {
        m_turretPowerMap.put(0.0,  0.00);
        m_turretPowerMap.put(2.0,  0.03);
        m_turretPowerMap.put(5.0,  0.07);
        m_turretPowerMap.put(10.0, 0.14);
        m_turretPowerMap.put(20.0, 0.25);
        m_turretPowerMap.put(45.0, 0.40);
        m_turretPowerMap.put(90.0, 0.50);
        m_turretPowerMap.put(180.0, 0.50);
    }

    @Override
    public void periodic() {
        // --- Hood PID live tuning ---
        m_hoodMaxPercent = SmartDashboard.getNumber("Hood/MaxPercent", m_hoodMaxPercent);
        double hoodP = SmartDashboard.getNumber("Hood/kP", m_lastHoodP);
        double hoodI = SmartDashboard.getNumber("Hood/kI", m_lastHoodI);
        double hoodD = SmartDashboard.getNumber("Hood/kD", m_lastHoodD);
        if (hoodP != m_lastHoodP || hoodI != m_lastHoodI || hoodD != m_lastHoodD) {
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = hoodP; slot0.kI = hoodI; slot0.kD = hoodD;
            m_hoodMotor.getConfigurator().apply(slot0);
            m_lastHoodP = hoodP; m_lastHoodI = hoodI; m_lastHoodD = hoodD;
        }
    }

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
     * Commands the turret toward a target physical angle.
     *
     * Convention:
     *   180° = straight backward (home / encoder-zero position)
     *    90° = left side of robot
     *   270° = right side of robot
     *
     * The error (target − current) is looked up in the power table to produce a
     * duty-cycle output. The SparkFlex's soft limits act as a hardware backstop.
     *
     * If the turret runs away from the target on first test, negate {@code power}
     * below — the motor polarity depends on physical wiring.
     */
    public void setTurretTargetDeg(double targetDeg) {
        // Clamp target to the physically reachable range (90° – 270°)
        double clampedTarget = MathUtil.clamp(targetDeg, 90.0, 270.0);
        m_targetTurretDeg = clampedTarget;

        double currentDeg = getTurretAngleDeg();
        double errorDeg = clampedTarget - currentDeg;

        // Software guard: don't command further into a hard stop
        double currentRotations = getTurretRotations();
        if (currentRotations >= ShooterConstants.kTurretMaxRotations && errorDeg > 0) {
            m_turretMotor.set(0);
            return;
        }
        if (currentRotations <= ShooterConstants.kTurretMinRotations && errorDeg < 0) {
            m_turretMotor.set(0);
            return;
        }

        // Lookup: magnitude from table, direction from sign of error
        double power = m_turretPowerMap.get(Math.abs(errorDeg)) * Math.signum(errorDeg);
        m_turretMotor.set(power);
    }

    /** Zeros the hood encoder at the current position. */
    public void zeroHoodEncoder() {
        m_hoodMotor.setPosition(0.0);
    }

    /** Stops only the flywheels. */
    public void stopFlywheels() {
        m_leftFlywheel.stopMotor();
        m_rightFlywheel.stopMotor();
    }

    /** Open-loop turret control for manual joystick override; capped at 50% output. */
    public void runTurretManual(double joystickInput) {
        m_turretMotor.set(joystickInput * 0.5);
    }

    /** Open-loop hood control, clamped to SmartDashboard MaxPercent. */
    public void setHoodPercent(double percent) {
        m_hoodMotor.setControl(m_dutyCycleRequest.withOutput(
                MathUtil.clamp(percent, -m_hoodMaxPercent, m_hoodMaxPercent)));
    }

    public void stopShooter() {
        m_leftFlywheel.stopMotor();
        m_rightFlywheel.stopMotor();
        setTurretTargetDeg(180.0); // return turret to home
        setHoodPosition(0.0);
    }

    // ==========================================================
    // GETTERS
    // ==========================================================

    public double getTargetRps()       { return m_targetFlywheelRps; }
    public double getTargetHood()      { return m_targetHoodRotations; }
    public double getTargetTurretDeg() { return m_targetTurretDeg; }
    public double getHoodCurrent() { return m_hoodMotor.getStatorCurrent().getValueAsDouble(); }

    public double getFlywheelRps() {
        return m_leftFlywheel.getVelocity().getValueAsDouble();
    }

    public double getTurretRotations() {
        return m_turretEncoder.getPosition();
    }

    /**
     * Turret angle in physical degrees:
     *   180° = straight backward (home, encoder = 0)
     *    90° = left side  (encoder = kTurretMinRotations)
     *   270° = right side (encoder = kTurretMaxRotations)
     */
    public double getTurretAngleDeg() {
        return (getTurretRotations() / ShooterConstants.kTurretMotorRotationsPerTurretRevolution)
                * 360.0 + 180.0;
    }

    public double getHoodRotations() {
        return m_hoodMotor.getPosition().getValueAsDouble();
    }

    public boolean isReadyToFire() {
        boolean flywheelsReady = Math.abs(getFlywheelRps() - m_targetFlywheelRps)
                <= ShooterConstants.kFlywheelToleranceRPS;
        boolean turretReady = Math.abs(getTurretAngleDeg() - m_targetTurretDeg)
                <= ShooterConstants.kTurretToleranceDeg;
        boolean hoodReady = Math.abs(getHoodRotations() - m_targetHoodRotations)
                <= ShooterConstants.kHoodToleranceRotations;

        if (m_targetFlywheelRps == 0) return false;

        return flywheelsReady && turretReady && hoodReady;
    }
}
