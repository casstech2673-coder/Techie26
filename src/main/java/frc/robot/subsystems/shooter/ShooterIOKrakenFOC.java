// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOKrakenFOC implements ShooterIO {

  private final TalonFX topMotor = new TalonFX(ShooterConstants.kTopMotorCanId);
  private final TalonFX bottomMotor = new TalonFX(ShooterConstants.kBottomMotorCanId);

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final NeutralOut stopRequest = new NeutralOut();

  private final StatusSignal<AngularVelocity> topVel = topMotor.getVelocity();
  private final StatusSignal<AngularVelocity> bottomVel = bottomMotor.getVelocity();
  private final StatusSignal<Voltage> topVolts = topMotor.getMotorVoltage();
  private final StatusSignal<Voltage> bottomVolts = bottomMotor.getMotorVoltage();
  private final StatusSignal<Current> topStator = topMotor.getStatorCurrent();
  private final StatusSignal<Current> bottomStator = bottomMotor.getStatorCurrent();
  private final StatusSignal<Current> topSupply = topMotor.getSupplyCurrent();
  private final StatusSignal<Current> bottomSupply = bottomMotor.getSupplyCurrent();
  private final StatusSignal<Temperature> topTemp = topMotor.getDeviceTemp();
  private final StatusSignal<Temperature> bottomTemp = bottomMotor.getDeviceTemp();

  private double prevTopRPM = 0.0;
  private double setpointRPM = 0.0;
  private boolean recovering = false;

  public ShooterIOKrakenFOC() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kStallCurrentAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.kShooterGearRatio;
    config.Slot0.kS = ShooterConstants.kFlywheelKs;
    config.Slot0.kV = ShooterConstants.kFlywheelKv;
    config.Slot0.kA = ShooterConstants.kFlywheelKa;
    config.Slot0.kP = ShooterConstants.kFlywheelKp;
    topMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        250, topVel, bottomVel, topStator, bottomStator, topSupply, bottomSupply);
    BaseStatusSignal.setUpdateFrequencyForAll(50, topVolts, bottomVolts, topTemp, bottomTemp);
    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    boolean topOk =
        BaseStatusSignal.refreshAll(topVel, topVolts, topStator, topSupply, topTemp).isOK();
    boolean bottomOk =
        BaseStatusSignal.refreshAll(bottomVel, bottomVolts, bottomStator, bottomSupply, bottomTemp)
            .isOK();

    double topRPM = topVel.getValueAsDouble() * 60.0;
    inputs.flywheelVelocityRPM[0] = topRPM;
    inputs.flywheelVelocityRPM[1] = bottomVel.getValueAsDouble() * 60.0;
    inputs.flywheelAppliedVolts[0] = topVolts.getValueAsDouble();
    inputs.flywheelAppliedVolts[1] = bottomVolts.getValueAsDouble();
    inputs.statorCurrentAmps[0] = topStator.getValueAsDouble();
    inputs.statorCurrentAmps[1] = bottomStator.getValueAsDouble();
    inputs.supplyCurrentAmps[0] = topSupply.getValueAsDouble();
    inputs.supplyCurrentAmps[1] = bottomSupply.getValueAsDouble();
    inputs.motorTempCelsius[0] = topTemp.getValueAsDouble();
    inputs.motorTempCelsius[1] = bottomTemp.getValueAsDouble();
    inputs.motorConnected[0] = topOk;
    inputs.motorConnected[1] = bottomOk;

    if (prevTopRPM - topRPM > ShooterConstants.kVelocityRecoveryThresholdRPM) recovering = true;
    if (recovering && Math.abs(topRPM - setpointRPM) < ShooterConstants.kReadyToFireRPMTolerance) {
      inputs.velocityRecovered = true;
      recovering = false;
    } else {
      inputs.velocityRecovered = false;
    }
    prevTopRPM = topRPM;
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    setpointRPM = rpm;
    topMotor.setControl(velocityRequest.withVelocity(rpm / 60.0));
    bottomMotor.setControl(velocityRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    topMotor.setControl(voltageRequest.withOutput(volts));
    bottomMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setFlywheelVelocityIndependent(double topRPM, double bottomRPM) {
    topMotor.setControl(velocityRequest.withVelocity(topRPM / 60.0));
    bottomMotor.setControl(velocityRequest.withVelocity(bottomRPM / 60.0));
  }

  @Override
  public void stop() {
    topMotor.setControl(stopRequest);
    bottomMotor.setControl(stopRequest);
  }
}
