// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Operator manual override for turret, hood, and flywheel.
 *
 * <p>Requires Superstructure — interrupts any active AimAndShootCommand or PassCommand when
 * scheduled. Hardware limits (turret soft-stops, hood min/max) are still enforced.
 *
 * <p>Operator RT is a "dumb" fixed-RPM shot with no vision — useful for clearing jams or shooting
 * when the Limelight is unavailable. Intake is still routed through IntakeMode so operator
 * LT/LB/Y/A buttons work concurrently.
 */
public class ManualOperatorCommand extends Command {

  private static final double kLoopPeriodSec = 0.020;

  private final Superstructure superstructure;
  private final DoubleSupplier turretStick; // operator right stick X (deadband applied here)
  private final DoubleSupplier hoodStick; // operator left stick Y, pre-inverted by caller
  private final BooleanSupplier manualShoot; // operator RT

  // Hood setpoint persists across loops so the operator nudges it incrementally.
  private double manualHoodAngleDeg = ShooterConstants.kHoodIdleAngleDeg;

  public ManualOperatorCommand(
      Superstructure superstructure,
      DoubleSupplier turretStick,
      DoubleSupplier hoodStick,
      BooleanSupplier manualShoot) {
    this.superstructure = superstructure;
    this.turretStick = turretStick;
    this.hoodStick = hoodStick;
    this.manualShoot = manualShoot;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    manualHoodAngleDeg = ShooterConstants.kHoodIdleAngleDeg;
  }

  @Override
  public void execute() {
    // Turret: stick input → degrees to rotate this loop.
    double turretInput = MathUtil.applyDeadband(turretStick.getAsDouble(), 0.1);
    double turretDeltaDeg =
        turretInput * ShooterConstants.kManualTurretMaxDegPerSec * kLoopPeriodSec;

    // Hood: incrementally adjust stored setpoint; clamped to hardware limits.
    double hoodInput = MathUtil.applyDeadband(hoodStick.getAsDouble(), 0.1);
    manualHoodAngleDeg =
        MathUtil.clamp(
            manualHoodAngleDeg + hoodInput * ShooterConstants.kManualHoodDegPerSec * kLoopPeriodSec,
            ShooterConstants.kHoodMinAngleDeg,
            ShooterConstants.kHoodMaxAngleDeg);

    double flywheelRPM = manualShoot.getAsBoolean() ? ShooterConstants.kManualShootRPM : 0.0;

    superstructure.setManualInputs(turretDeltaDeg, manualHoodAngleDeg, flywheelRPM);
    superstructure.setGoal(SuperstructureState.MANUAL_CONTROL);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setGoal(SuperstructureState.IDLE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
