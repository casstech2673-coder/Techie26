// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

  private static final double kNeoMOI = 0.0005;

  private final DCMotorSim intakeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), kNeoMOI, 1.0), DCMotor.getNEO(1));
  private final DCMotorSim hopperSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), kNeoMOI, 1.0),
          DCMotor.getNeoVortex(1));
  private final DCMotorSim kickerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), kNeoMOI, 1.0), DCMotor.getNEO(1));

  private double intakeApplied = 0.0;
  private double hopperApplied = 0.0;
  private double kickerApplied = 0.0;

  private boolean simIntakeTripped = false;
  private boolean simHopperTripped = false;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.setInputVoltage(intakeApplied * 12.0);
    hopperSim.setInputVoltage(hopperApplied * 12.0);
    kickerSim.setInputVoltage(kickerApplied * 12.0);
    intakeSim.update(0.02);
    hopperSim.update(0.02);
    kickerSim.update(0.02);

    inputs.intakeVelocityRPM = intakeSim.getAngularVelocityRPM();
    inputs.hopperVelocityRPM = hopperSim.getAngularVelocityRPM();
    inputs.kickerVelocityRPM = kickerSim.getAngularVelocityRPM();
    inputs.intakeAppliedOutput = intakeApplied;
    inputs.hopperAppliedOutput = hopperApplied;
    inputs.kickerAppliedOutput = kickerApplied;
    inputs.intakeCurrentAmps = Math.abs(intakeSim.getCurrentDrawAmps());
    inputs.hopperCurrentAmps = Math.abs(hopperSim.getCurrentDrawAmps());
    inputs.kickerCurrentAmps = Math.abs(kickerSim.getCurrentDrawAmps());
    inputs.intakeBeamBreakTripped = simIntakeTripped;
    inputs.hopperBeamBreakTripped = simHopperTripped;
    inputs.motorConnected[0] = true;
    inputs.motorConnected[1] = true;
    inputs.motorConnected[2] = true;
  }

  @Override
  public void setIntakeOutput(double output) {
    intakeApplied = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setHopperOutput(double output) {
    hopperApplied = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void setKickerOutput(double output) {
    kickerApplied = MathUtil.clamp(output, -1.0, 1.0);
  }

  @Override
  public void stopAll() {
    intakeApplied = 0.0;
    hopperApplied = 0.0;
    kickerApplied = 0.0;
  }

  public void setIntakeBeamBreak(boolean tripped) {
    simIntakeTripped = tripped;
  }

  public void setHopperBeamBreak(boolean tripped) {
    simHopperTripped = tripped;
  }
}
