// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/**
 * Real-hardware turret IO using a NEO Vortex on a SparkFlex (CAN {@link
 * TurretConstants#kTurretMotorCanId}).
 *
 * <p>Position PID runs on the RIO (WPILib {@link PIDController}) so that AdvantageKit can replay
 * control logic from logs. The SparkFlex receives only voltage commands.
 *
 * <p>The built-in relative encoder is software-seeded at startup via {@link
 * #seedStartupPosition(double)} — no homing sequence needed.
 */
public class TurretIOSparkFlex implements TurretIO {

  private final SparkFlex motor =
      new SparkFlex(TurretConstants.kTurretMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  private final PIDController pid =
      new PIDController(TurretConstants.kTurretKp, 0, TurretConstants.kTurretKd);

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public TurretIOSparkFlex() {
    var config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    // Position conversion: motor rotations → degrees of turret travel
    config
        .encoder
        .positionConversionFactor(360.0 / TurretConstants.kTurretGearRatio)
        .velocityConversionFactor(360.0 / TurretConstants.kTurretGearRatio / 60.0);

    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(motor, encoder::getPosition, (v) -> inputs.currentAngleDeg = v);
    ifOk(motor, encoder::getVelocity, (v) -> inputs.velocityDegPerSec = v);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (v) -> inputs.appliedVolts = v[0] * v[1]);
    ifOk(motor, motor::getOutputCurrent, (v) -> inputs.statorCurrentAmps = v);
    ifOk(motor, motor::getMotorTemperature, (v) -> inputs.motorTempCelsius = v);
    inputs.motorConnected = connectedDebounce.calculate(!sparkStickyFault);
    inputs.encoderConnected = inputs.motorConnected;
    inputs.atForwardLimit = false; // no hardware limit switches configured
    inputs.atReverseLimit = false;
  }

  @Override
  public void setTargetAngle(double angleDeg) {
    double clamped =
        MathUtil.clamp(
            angleDeg, TurretConstants.kSoftMinAngleDeg, TurretConstants.kSoftMaxAngleDeg);
    double output = MathUtil.clamp(pid.calculate(encoder.getPosition(), clamped), -12.0, 12.0);
    motor.setVoltage(output);
  }

  @Override
  public void setVoltage(double volts) {
    pid.reset();
    motor.setVoltage(volts);
  }

  @Override
  public void holdPosition() {
    setTargetAngle(encoder.getPosition());
  }

  @Override
  public void stop() {
    pid.reset();
    motor.stopMotor();
  }

  @Override
  public void zeroEncoder() {
    encoder.setPosition(TurretConstants.kMinAngleDeg);
  }

  @Override
  public void seedStartupPosition(double angleDeg) {
    encoder.setPosition(angleDeg);
  }

  @Override
  public void setPIDGains(double kP, double kD) {
    pid.setP(kP);
    pid.setD(kD);
  }
}
