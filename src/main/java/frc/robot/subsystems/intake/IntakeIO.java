// Copyright (c) 2026 FRC Team 2673
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the intake roller, hopper/kicker belt, and arm pivot. */
public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public double intakeVelocityRPM = 0.0;
    /** Combined hopper+kicker motor (CAN 41). Speed setpoint changes between modes. */
    public double hopperVelocityRPM = 0.0;

    public double intakeAppliedOutput = 0.0;
    public double hopperAppliedOutput = 0.0;
    public double intakeCurrentAmps = 0.0;
    public double hopperCurrentAmps = 0.0;
    public double armPositionDeg = 0.0;
    /** [0]=intake, [1]=hopper, [2]=arm */
    public boolean[] motorConnected = new boolean[] {false, false, false};
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  /** Intake roller. Positive = intake direction. */
  default void setIntakeOutput(double output) {}

  /**
   * Hopper/kicker motor (CAN 41). Use {@link IntakeConstants#kHopperFeedSpeed} for indexing, {@link
   * IntakeConstants#kHopperKickSpeed} for shooting.
   */
  default void setHopperOutput(double output) {}

  default void stopAll() {}

  /** Arm pivot. 0° = down. Implementations must clamp to hardware limits. */
  default void setArmPosition(double angleDeg) {}
}
