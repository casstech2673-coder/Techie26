// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), 0.5, TurretConstants.kTurretGearRatio),
          DCMotor.getKrakenX60Foc(1));

  private final PIDController pid =
      new PIDController(TurretConstants.kTurretKp, 0, TurretConstants.kTurretKd);

  private double appliedVolts = 0.0;
  private boolean positionControl = false;
  private double targetAngleDeg = 0.0;
  // Track absolute position since DCMotorSim only gives relative
  private double absoluteAngleDeg = 0.0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (positionControl) {
      appliedVolts = MathUtil.clamp(pid.calculate(absoluteAngleDeg, targetAngleDeg), -12.0, 12.0);
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);
    absoluteAngleDeg += Units.radiansToDegrees(sim.getAngularVelocityRadPerSec()) * 0.02;

    inputs.currentAngleDeg = absoluteAngleDeg;
    inputs.velocityDegPerSec = Units.radiansToDegrees(sim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.motorTempCelsius = 25.0;
    inputs.atForwardLimit = absoluteAngleDeg <= TurretConstants.kMinAngleDeg + 1.0;
    inputs.atReverseLimit = absoluteAngleDeg >= TurretConstants.kMaxAngleDeg - 1.0;
    inputs.motorConnected = true;
    inputs.encoderConnected = true;
  }

  @Override
  public void setTargetAngle(double angleDeg) {
    positionControl = true;
    targetAngleDeg =
        MathUtil.clamp(
            angleDeg, TurretConstants.kSoftMinAngleDeg, TurretConstants.kSoftMaxAngleDeg);
  }

  @Override
  public void setVoltage(double volts) {
    positionControl = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void holdPosition() {
    setTargetAngle(absoluteAngleDeg);
  }

  @Override
  public void stop() {
    positionControl = false;
    appliedVolts = 0.0;
  }

  @Override
  public void zeroEncoder() {
    absoluteAngleDeg = TurretConstants.kMinAngleDeg;
  }
}
