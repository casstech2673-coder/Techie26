// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

  private static final double kMOI = 0.003; // kg·m²

  private final DCMotorSim leftSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), kMOI, ShooterConstants.kShooterGearRatio),
          DCMotor.getKrakenX60Foc(1));
  private final DCMotorSim rightSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), kMOI, ShooterConstants.kShooterGearRatio),
          DCMotor.getKrakenX60Foc(1));

  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(ShooterConstants.kFlywheelKs, ShooterConstants.kFlywheelKv);
  private final PIDController leftPID = new PIDController(ShooterConstants.kFlywheelKp, 0, 0);
  private final PIDController rightPID = new PIDController(ShooterConstants.kFlywheelKp, 0, 0);

  private double leftApplied = 0.0;
  private double rightApplied = 0.0;
  private boolean closedLoop = false;
  private double leftSetpoint = 0.0;
  private double rightSetpoint = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      leftApplied =
          MathUtil.clamp(
              ff.calculate(leftSetpoint / 60.0)
                  + leftPID.calculate(leftSim.getAngularVelocityRPM(), leftSetpoint),
              -12.0,
              12.0);
      rightApplied =
          MathUtil.clamp(
              ff.calculate(rightSetpoint / 60.0)
                  + rightPID.calculate(rightSim.getAngularVelocityRPM(), rightSetpoint),
              -12.0,
              12.0);
    }
    leftSim.setInputVoltage(leftApplied);
    rightSim.setInputVoltage(rightApplied);
    leftSim.update(0.02);
    rightSim.update(0.02);

    inputs.flywheelVelocityRPM[0] = leftSim.getAngularVelocityRPM();
    inputs.flywheelVelocityRPM[1] = rightSim.getAngularVelocityRPM();
    inputs.flywheelAppliedVolts[0] = leftApplied;
    inputs.flywheelAppliedVolts[1] = rightApplied;
    inputs.statorCurrentAmps[0] = Math.abs(leftSim.getCurrentDrawAmps());
    inputs.statorCurrentAmps[1] = Math.abs(rightSim.getCurrentDrawAmps());
    inputs.supplyCurrentAmps[0] = inputs.statorCurrentAmps[0];
    inputs.supplyCurrentAmps[1] = inputs.statorCurrentAmps[1];
    inputs.motorTempCelsius[0] = 25.0;
    inputs.motorTempCelsius[1] = 25.0;
    inputs.motorConnected[0] = true;
    inputs.motorConnected[1] = true;
    inputs.velocityRecovered = false;
    inputs.hoodAngleDeg = 0.0;
    inputs.hoodAppliedOutput = 0.0;
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    closedLoop = true;
    leftSetpoint = rpm;
    rightSetpoint = rpm;
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    closedLoop = false;
    leftApplied = MathUtil.clamp(volts, -12.0, 12.0);
    rightApplied = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setFlywheelVelocityIndependent(double leftRPM, double rightRPM) {
    closedLoop = true;
    leftSetpoint = leftRPM;
    rightSetpoint = rightRPM;
  }

  @Override
  public void stop() {
    closedLoop = false;
    leftApplied = 0.0;
    rightApplied = 0.0;
  }
}
