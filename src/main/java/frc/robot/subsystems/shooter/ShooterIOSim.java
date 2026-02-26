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

  private final DCMotorSim topSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), kMOI, ShooterConstants.kShooterGearRatio),
          DCMotor.getKrakenX60Foc(1));
  private final DCMotorSim bottomSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), kMOI, ShooterConstants.kShooterGearRatio),
          DCMotor.getKrakenX60Foc(1));

  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(ShooterConstants.kFlywheelKs, ShooterConstants.kFlywheelKv);
  private final PIDController topPID = new PIDController(ShooterConstants.kFlywheelKp, 0, 0);
  private final PIDController bottomPID = new PIDController(ShooterConstants.kFlywheelKp, 0, 0);

  private double topApplied = 0.0;
  private double bottomApplied = 0.0;
  private boolean closedLoop = false;
  private double topSetpoint = 0.0;
  private double bottomSetpoint = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      topApplied =
          MathUtil.clamp(
              ff.calculate(topSetpoint / 60.0)
                  + topPID.calculate(topSim.getAngularVelocityRPM(), topSetpoint),
              -12.0,
              12.0);
      bottomApplied =
          MathUtil.clamp(
              ff.calculate(bottomSetpoint / 60.0)
                  + bottomPID.calculate(bottomSim.getAngularVelocityRPM(), bottomSetpoint),
              -12.0,
              12.0);
    }
    topSim.setInputVoltage(topApplied);
    bottomSim.setInputVoltage(bottomApplied);
    topSim.update(0.02);
    bottomSim.update(0.02);

    inputs.flywheelVelocityRPM[0] = topSim.getAngularVelocityRPM();
    inputs.flywheelVelocityRPM[1] = bottomSim.getAngularVelocityRPM();
    inputs.flywheelAppliedVolts[0] = topApplied;
    inputs.flywheelAppliedVolts[1] = bottomApplied;
    inputs.statorCurrentAmps[0] = Math.abs(topSim.getCurrentDrawAmps());
    inputs.statorCurrentAmps[1] = Math.abs(bottomSim.getCurrentDrawAmps());
    inputs.supplyCurrentAmps[0] = inputs.statorCurrentAmps[0];
    inputs.supplyCurrentAmps[1] = inputs.statorCurrentAmps[1];
    inputs.motorTempCelsius[0] = 25.0;
    inputs.motorTempCelsius[1] = 25.0;
    inputs.motorConnected[0] = true;
    inputs.motorConnected[1] = true;
    inputs.velocityRecovered = false;
  }

  @Override
  public void setFlywheelVelocity(double rpm) {
    closedLoop = true;
    topSetpoint = rpm;
    bottomSetpoint = rpm;
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    closedLoop = false;
    topApplied = MathUtil.clamp(volts, -12.0, 12.0);
    bottomApplied = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setFlywheelVelocityIndependent(double topRPM, double bottomRPM) {
    closedLoop = true;
    topSetpoint = topRPM;
    bottomSetpoint = bottomRPM;
  }

  @Override
  public void stop() {
    closedLoop = false;
    topApplied = 0.0;
    bottomApplied = 0.0;
  }
}
