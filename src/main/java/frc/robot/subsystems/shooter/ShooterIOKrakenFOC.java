// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Real-hardware shooter IO.
 *
 * <p><b>Flywheels</b> — Two Kraken X60 (FOC). Left (CAN {@link ShooterConstants#kLeftMotorCanId})
 * is the leader; right (CAN {@link ShooterConstants#kRightMotorCanId}) is inverted and commanded to
 * the same setpoint each cycle (software synchronisation). Both motors share the same closed- loop
 * gains; the right motor's inversion config handles opposite physical spin direction.
 *
 * <p><b>Hood</b> — Kraken X44 (CAN {@link ShooterConstants#kHoodMotorCanId}) using MotionMagic
 * position control. Encoder is seeded to 0° at boot (startup = hood fully down).
 */
public class ShooterIOKrakenFOC implements ShooterIO {

  // ── Flywheel motors ────────────────────────────────────────────────────────
  private final TalonFX leftMotor = new TalonFX(ShooterConstants.kLeftMotorCanId);
  private final TalonFX rightMotor = new TalonFX(ShooterConstants.kRightMotorCanId);

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final NeutralOut stopRequest = new NeutralOut();

  // ── Left motor status signals ─────────────────────────────────────────────
  private final StatusSignal<AngularVelocity> leftVel = leftMotor.getVelocity();
  private final StatusSignal<Voltage> leftVolts = leftMotor.getMotorVoltage();
  private final StatusSignal<Current> leftStator = leftMotor.getStatorCurrent();
  private final StatusSignal<Current> leftSupply = leftMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> leftTemp = leftMotor.getDeviceTemp();

  // ── Right motor status signals ────────────────────────────────────────────
  private final StatusSignal<AngularVelocity> rightVel = rightMotor.getVelocity();
  private final StatusSignal<Voltage> rightVolts = rightMotor.getMotorVoltage();
  private final StatusSignal<Current> rightStator = rightMotor.getStatorCurrent();
  private final StatusSignal<Current> rightSupply = rightMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> rightTemp = rightMotor.getDeviceTemp();

  // ── Hood motor (Kraken X44) ───────────────────────────────────────────────
  private final TalonFX hoodMotor = new TalonFX(ShooterConstants.kHoodMotorCanId);
  private final MotionMagicVoltage hoodPositionRequest =
      new MotionMagicVoltage(0.0).withUpdateFreqHz(0);
  private final StatusSignal<Angle> hoodPos = hoodMotor.getPosition();
  private final StatusSignal<Voltage> hoodVolts = hoodMotor.getMotorVoltage();

  // ── State ─────────────────────────────────────────────────────────────────
  private double prevLeftRPM = 0.0;
  private double setpointRPM = 0.0;
  private boolean recovering = false;

  public ShooterIOKrakenFOC() {
    // ── Flywheel config (shared base) ──────────────────────────────────────
    var fxConfig = new TalonFXConfiguration();
    fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    fxConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kStallCurrentAmps;
    fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    fxConfig.Feedback.SensorToMechanismRatio = ShooterConstants.kShooterGearRatio;
    fxConfig.Slot0.kS = ShooterConstants.kFlywheelKs;
    fxConfig.Slot0.kV = ShooterConstants.kFlywheelKv;
    fxConfig.Slot0.kA = ShooterConstants.kFlywheelKa;
    fxConfig.Slot0.kP = ShooterConstants.kFlywheelKp;
    leftMotor.getConfigurator().apply(fxConfig);

    // Right motor: same config, opposite inversion so both push ball the same direction.
    fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotor.getConfigurator().apply(fxConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        250, leftVel, rightVel, leftStator, rightStator, leftSupply, rightSupply);
    BaseStatusSignal.setUpdateFrequencyForAll(50, leftVolts, rightVolts, leftTemp, rightTemp);
    leftMotor.optimizeBusUtilization();
    rightMotor.optimizeBusUtilization();

    // ── Hood config ────────────────────────────────────────────────────────
    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.CurrentLimits.StatorCurrentLimit = 20.0;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.kHoodGearRatio;
    hoodConfig.Slot0.kP = ShooterConstants.kHoodKp;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ShooterConstants.kHoodMaxAngleDeg / 360.0;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ShooterConstants.kHoodMinAngleDeg / 360.0;
    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 60.0 / 360.0; // 60 °/s in rot/s
    hoodConfig.MotionMagic.MotionMagicAcceleration = 120.0 / 360.0;
    hoodMotor.getConfigurator().apply(hoodConfig);
    hoodMotor.setPosition(0.0); // seed to 0° — startup position is hood fully down

    BaseStatusSignal.setUpdateFrequencyForAll(100, hoodPos, hoodVolts);
    hoodMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    boolean leftOk =
        BaseStatusSignal.refreshAll(leftVel, leftVolts, leftStator, leftSupply, leftTemp).isOK();
    boolean rightOk =
        BaseStatusSignal.refreshAll(rightVel, rightVolts, rightStator, rightSupply, rightTemp)
            .isOK();
    BaseStatusSignal.refreshAll(hoodPos, hoodVolts);

    double leftRPM = leftVel.getValueAsDouble() * 60.0;
    inputs.flywheelVelocityRPM[0] = leftRPM;
    inputs.flywheelVelocityRPM[1] = rightVel.getValueAsDouble() * 60.0;
    inputs.flywheelAppliedVolts[0] = leftVolts.getValueAsDouble();
    inputs.flywheelAppliedVolts[1] = rightVolts.getValueAsDouble();
    inputs.statorCurrentAmps[0] = leftStator.getValueAsDouble();
    inputs.statorCurrentAmps[1] = rightStator.getValueAsDouble();
    inputs.supplyCurrentAmps[0] = leftSupply.getValueAsDouble();
    inputs.supplyCurrentAmps[1] = rightSupply.getValueAsDouble();
    inputs.motorTempCelsius[0] = leftTemp.getValueAsDouble();
    inputs.motorTempCelsius[1] = rightTemp.getValueAsDouble();
    inputs.motorConnected[0] = leftOk;
    inputs.motorConnected[1] = rightOk;

    if (prevLeftRPM - leftRPM > ShooterConstants.kVelocityRecoveryThresholdRPM) recovering = true;
    if (recovering && Math.abs(leftRPM - setpointRPM) < ShooterConstants.kReadyToFireRPMTolerance) {
      inputs.velocityRecovered = true;
      recovering = false;
    } else {
      inputs.velocityRecovered = false;
    }
    prevLeftRPM = leftRPM;

    inputs.hoodAngleDeg = hoodPos.getValueAsDouble() * 360.0;
    inputs.hoodAppliedOutput = hoodVolts.getValueAsDouble();
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    setpointRPM = rpm;
    leftMotor.setControl(velocityRequest.withVelocity(rpm / 60.0));
    rightMotor.setControl(velocityRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    leftMotor.setControl(voltageRequest.withOutput(volts));
    rightMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setFlywheelVelocityIndependent(double leftRPM, double rightRPM) {
    leftMotor.setControl(velocityRequest.withVelocity(leftRPM / 60.0));
    rightMotor.setControl(velocityRequest.withVelocity(rightRPM / 60.0));
  }

  @Override
  public void stop() {
    leftMotor.setControl(stopRequest);
    rightMotor.setControl(stopRequest);
  }

  @Override
  public void setHoodAngle(double angleDeg) {
    double clamped =
        MathUtil.clamp(
            angleDeg, ShooterConstants.kHoodMinAngleDeg, ShooterConstants.kHoodMaxAngleDeg);
    hoodMotor.setControl(hoodPositionRequest.withPosition(clamped / 360.0));
  }

  @Override
  public void setPIDGains(double kS, double kV, double kA, double kP) {
    var slot0 = new Slot0Configs();
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kP = kP;
    leftMotor.getConfigurator().apply(slot0);
    rightMotor.getConfigurator().apply(slot0);
  }
}
