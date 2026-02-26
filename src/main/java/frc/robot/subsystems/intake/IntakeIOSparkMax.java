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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.DoubleSupplier;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax intakeMotor =
      new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
  private final SparkFlex hopperMotor =
      new SparkFlex(IntakeConstants.kHopperMotorCanId, MotorType.kBrushless);
  private final SparkMax kickerMotor =
      new SparkMax(IntakeConstants.kKickerMotorCanId, MotorType.kBrushless);

  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private final RelativeEncoder hopperEncoder = hopperMotor.getEncoder();
  private final RelativeEncoder kickerEncoder = kickerMotor.getEncoder();

  private final DigitalInput intakeBeamBreak =
      new DigitalInput(IntakeConstants.kIntakeBeamBreakPort);
  private final DigitalInput hopperBeamBreak =
      new DigitalInput(IntakeConstants.kHopperBeamBreakPort);

  private final Debouncer intakeConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer hopperConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer kickerConnectedDebounce =
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

    var kickerConfig = new SparkMaxConfig();
    kickerConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.kKickerSmartCurrentLimit);
    kickerConfig
        .signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        kickerMotor,
        5,
        () ->
            kickerMotor.configure(
                kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
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
    ifOk(kickerMotor, kickerEncoder::getVelocity, (v) -> inputs.kickerVelocityRPM = v);
    ifOk(
        kickerMotor,
        new DoubleSupplier[] {kickerMotor::getAppliedOutput, kickerMotor::getBusVoltage},
        (v) -> inputs.kickerAppliedOutput = v[0]);
    ifOk(kickerMotor, kickerMotor::getOutputCurrent, (v) -> inputs.kickerCurrentAmps = v);
    inputs.motorConnected[2] = kickerConnectedDebounce.calculate(!sparkStickyFault);

    inputs.intakeBeamBreakTripped = !intakeBeamBreak.get();
    inputs.hopperBeamBreakTripped = !hopperBeamBreak.get();
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
  public void setKickerOutput(double output) {
    kickerMotor.set(output);
  }

  @Override
  public void stopAll() {
    intakeMotor.stopMotor();
    hopperMotor.stopMotor();
    kickerMotor.stopMotor();
  }
}
