// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX motor = new TalonFX(TurretConstants.kTurretMotorCanId);
  private final CANcoder encoder = new CANcoder(TurretConstants.kTurretEncoderCanId);

  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(0.0).withUpdateFreqHz(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final NeutralOut stopRequest = new NeutralOut();

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> volts = motor.getMotorVoltage();
  private final StatusSignal<Current> stator = motor.getStatorCurrent();
  private final StatusSignal<Temperature> temp = motor.getDeviceTemp();
  private final StatusSignal<ForwardLimitValue> fwdLimit = motor.getForwardLimit();
  private final StatusSignal<ReverseLimitValue> revLimit = motor.getReverseLimit();

  public TurretIOTalonFX() {
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    encoder.getConfigurator().apply(encoderConfig);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = TurretConstants.kTurretGearRatio;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = TurretConstants.kTurretEncoderCanId;
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.kSoftMaxAngleDeg / 360.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.kSoftMinAngleDeg / 360.0;
    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kTurretMaxVelocityDegS / 360.0;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.kTurretMaxAccelDegS2 / 360.0;
    config.Slot0.kP = TurretConstants.kTurretKp;
    config.Slot0.kD = TurretConstants.kTurretKd;
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(250, position, velocity, stator);
    BaseStatusSignal.setUpdateFrequencyForAll(50, volts, temp, fwdLimit, revLimit);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    boolean motorOk =
        BaseStatusSignal.refreshAll(position, velocity, volts, stator, temp, fwdLimit, revLimit)
            .isOK();
    inputs.currentAngleDeg = position.getValueAsDouble() * 360.0;
    inputs.velocityDegPerSec = velocity.getValueAsDouble() * 360.0;
    inputs.appliedVolts = volts.getValueAsDouble();
    inputs.statorCurrentAmps = stator.getValueAsDouble();
    inputs.motorTempCelsius = temp.getValueAsDouble();
    inputs.atForwardLimit = fwdLimit.getValue() == ForwardLimitValue.ClosedToGround;
    inputs.atReverseLimit = revLimit.getValue() == ReverseLimitValue.ClosedToGround;
    inputs.motorConnected = motorOk;
    inputs.encoderConnected = motorOk;
  }

  @Override
  public void setTargetAngle(double angleDeg) {
    double clamped =
        MathUtil.clamp(
            angleDeg, TurretConstants.kSoftMinAngleDeg, TurretConstants.kSoftMaxAngleDeg);
    motor.setControl(positionRequest.withPosition(clamped / 360.0));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void holdPosition() {
    motor.setControl(positionRequest.withPosition(position.getValueAsDouble()));
  }

  @Override
  public void stop() {
    motor.setControl(stopRequest);
  }

  @Override
  public void zeroEncoder() {
    motor.setPosition(TurretConstants.kMinAngleDeg / 360.0);
  }
}
