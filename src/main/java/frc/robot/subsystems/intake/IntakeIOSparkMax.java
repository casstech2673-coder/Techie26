// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax intakeMotor =
      new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
  private final SparkFlex hopperMotor =
      new SparkFlex(IntakeConstants.kHopperMotorCanId, MotorType.kBrushless);
  private final SparkMax armMotor =
      new SparkMax(IntakeConstants.kArmMotorCanId, MotorType.kBrushless);

  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();
  private final RelativeEncoder armEncoder = armMotor.getEncoder();

  // RIO-side position PID for the arm pivot (keeps logic in Java for AKit replay)
  private final PIDController armPID = new PIDController(IntakeConstants.kArmKp, 0, 0);

  private final Debouncer intakeConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer hopperConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer armConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOSparkMax() {
    var intakeConfig = new SparkMaxConfig();
    intakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.kIntakeSmartCurrentLimit);
    intakeConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var hopperConfig = new SparkFlexConfig();
    hopperConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.kHopperSmartCurrentLimit);
    hopperConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        hopperMotor,
        5,
        () ->
            hopperMotor.configure(
                hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var armConfig = new SparkMaxConfig();
    armConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(IntakeConstants.kArmSmartCurrentLimit);
    // Position conversion: motor rotations → degrees of arm travel
    armConfig
        .encoder
        .positionConversionFactor(360.0 / IntakeConstants.kArmGearRatio)
        .velocityConversionFactor(360.0 / IntakeConstants.kArmGearRatio / 60.0);
    armConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        armMotor,
        5,
        () ->
            armMotor.configure(
                armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Seed arm encoder to 0° — robot always starts with arm fully down
    armEncoder.setPosition(0.0);
    armPID.setTolerance(1.0); // ±1° tolerance
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(intakeMotor, intakeEncoder::getVelocity, (v) -> inputs.intakeVelocityRPM = v);
    ifOk(
        intakeMotor,
        new DoubleSupplier[] {intakeMotor::getAppliedOutput, intakeMotor::getBusVoltage},
        (v) -> inputs.intakeAppliedOutput = v[0]);
    ifOk(intakeMotor, intakeMotor::getOutputCurrent, (v) -> inputs.intakeCurrentAmps = v);
    inputs.motorConnected[0] = intakeConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(hopperMotor, hopperEncoder::getVelocity, (v) -> inputs.hopperVelocityRPM = v);
    ifOk(
        hopperMotor,
        new DoubleSupplier[] {hopperMotor::getAppliedOutput, hopperMotor::getBusVoltage},
        (v) -> inputs.hopperAppliedOutput = v[0]);
    ifOk(hopperMotor, hopperMotor::getOutputCurrent, (v) -> inputs.hopperCurrentAmps = v);
    inputs.motorConnected[1] = hopperConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(armMotor, armEncoder::getPosition, (v) -> inputs.armPositionDeg = v);
    inputs.motorConnected[2] = armConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setIntakeOutput(double output) {
    intakeMotor.set(output);
  }

  @Override
  public void setHopperOutput(double output) {
    hopperMotor.set(output);
  }

  @Override
  public void setArmPosition(double angleDeg) {
    double clamped = MathUtil.clamp(angleDeg, 0.0, IntakeConstants.kArmMaxDeg);
    double output = MathUtil.clamp(armPID.calculate(armEncoder.getPosition(), clamped), -1.0, 1.0);
    armMotor.set(output);
  }

  @Override
  public void stopAll() {
    intakeMotor.stopMotor();
    hopperMotor.stopMotor();
    armMotor.stopMotor();
  }
}
